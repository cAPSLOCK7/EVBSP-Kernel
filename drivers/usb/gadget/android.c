/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <linux/usb/android.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
#include "f_nwm.h"
#else
#include "f_mass_storage.h"
#endif
#include "f_adb.h"

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
static const char longname[] = "Gadget Android for NWM";
#else
static const char longname[] = "Gadget Android";
#endif

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001
#define ADB_PRODUCT_ID	0x0002

struct android_dev {
	struct usb_composite_dev *cdev;

	int product_id;
	int adb_product_id;
	int version;

	int adb_enabled;
	int nluns;
};

static atomic_t adb_enable_excl;
static struct android_dev *_android_dev;

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
static struct usb_string strings_dev[] = {
	{STRING_MANUFACTURER,	"Windows MTP Device"},
	{STRING_PRODUCT,		"NEC Windows Media Device"},
	{STRING_SERIAL,			"313220417567"},
	{STRING_CONFIG,			"Self-powered"},
	{STRING_INTERFACE,		"MTP Device"},
/*	{STRING_INTERFACE_0,	"Mass Storage"}, */
/*	{STRING_INTERFACE_1,	"ADB"}, */
	{FC_NWM_STRING_OS_DESCRIPTOR,	"MSFT1000"},
	{}
};
#else
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	{  }			/* end of list */
};
#endif

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),

#ifdef USE_USB_IAD
	.bDeviceClass = USB_CLASS_MISC,
	.bDeviceSubClass = 0x02,
	.bDeviceProtocol = 0x01,
#else	/* !USE_USB_IAD */
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
#endif	/* USE_USB_IAD */

	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

void android_usb_set_connected(int connected)
{
	if (_android_dev && _android_dev->cdev && _android_dev->cdev->gadget) {
		if (connected)
			usb_gadget_connect(_android_dev->cdev->gadget);
		else
			usb_gadget_disconnect(_android_dev->cdev->gadget);
	}
}

int get_adb_enable()
{
	int		nret = -1;

	if (_android_dev)
		nret = _android_dev->adb_enabled;

	return nret;
}

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret;
	printk(KERN_DEBUG "android_bind_config\n");

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
	ret = nwm_function_add(c, dev->nluns);
#else
	ret = mass_storage_function_add(dev->cdev, c, dev->nluns);
#endif

	if (ret)
		return ret;
	return adb_function_add(dev->cdev, c);
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	adb_function_del(dev->cdev, c);

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
	nwm_function_del(c);
#else
	mass_storage_function_del(dev->cdev, c);
#endif

}

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0xFA, /* 500ma */
#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
	.iConfiguration = STRING_CONFIG,
#endif
};

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
#else
	int			gcnum;
#endif
	int			id;
	int			ret;

	printk(KERN_INFO "android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE) || defined(CONFIG_USB_GADGET_EMXX)
#else
	if (gadget->ops->wakeup)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
#endif

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		printk(KERN_ERR "usb_add_config failed\n");
		return ret;
	}

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
#else
	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}
#endif

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;

	return 0;
}

static int android_unbind(struct usb_composite_dev *cdev)
{
	struct usb_gadget	*gadget = cdev->gadget;

	usb_gadget_clear_selfpowered(gadget);

	usb_del_config(&android_config_driver);

	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.unbind		= android_unbind,
};

static void enable_adb(struct android_dev *dev, int enable)
{
	if (enable != dev->adb_enabled) {
		dev->adb_enabled = enable;
		adb_function_enable(enable);

		/* set product ID to the appropriate value */
		if (enable)
			device_desc.idProduct =
				__constant_cpu_to_le16(dev->adb_product_id);
		else
			device_desc.idProduct =
				__constant_cpu_to_le16(dev->product_id);
		if (dev->cdev)
			dev->cdev->desc.idProduct = device_desc.idProduct;

		/* force reenumeration */
		if (dev->cdev && dev->cdev->gadget &&
				dev->cdev->gadget->speed != USB_SPEED_UNKNOWN) {
			usb_gadget_disconnect(dev->cdev->gadget);
			msleep(10);
			usb_gadget_connect(dev->cdev->gadget);
		}
	}
}

static int adb_enable_open(struct inode *ip, struct file *fp)
{
	if (atomic_inc_return(&adb_enable_excl) != 1) {
		atomic_dec(&adb_enable_excl);
		return -EBUSY;
	}

	printk(KERN_INFO "enabling adb\n");
	enable_adb(_android_dev, 1);

	return 0;
}

static int adb_enable_release(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "disabling adb\n");
	enable_adb(_android_dev, 0);
	atomic_dec(&adb_enable_excl);
	return 0;
}

static const struct file_operations adb_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    adb_enable_open,
	.release = adb_enable_release,
};

static struct miscdevice adb_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb_enable",
	.fops = &adb_enable_fops,
};

static int android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_probe pdata: %p\n", pdata);

	if (pdata) {
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->adb_product_id)
			dev->adb_product_id = pdata->adb_product_id;
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
		dev->nluns = pdata->nluns;
	}

	return 0;
}

static int android_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", },
	.probe = android_probe,
	.remove = android_remove,
};

static int __init init(void)
{
	struct android_dev *dev;
	int ret;

	printk(KERN_INFO "android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = PRODUCT_ID;
	dev->adb_product_id = ADB_PRODUCT_ID;
	_android_dev = dev;

	ret = platform_driver_register(&android_platform_driver);
	if (ret)
		return ret;
	ret = misc_register(&adb_enable_device);
	if (ret) {
		platform_driver_unregister(&android_platform_driver);
		return ret;
	}
	ret = usb_composite_register(&android_usb_driver);
	if (ret) {
		misc_deregister(&adb_enable_device);
		platform_driver_unregister(&android_platform_driver);
	}
	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	misc_deregister(&adb_enable_device);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
