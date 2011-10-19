/*
 *  File Name       : drivers/watchdog/emxx_wdt.c
 *  Function        : EMMA Mobile series Watchdog driver
 *  Release Version : Ver 1.10
 *  Release Date    : 2010/04/01
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/ioport.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/fs.h>

#include <linux/uaccess.h>
#include <linux/io.h>

#include <mach/pwc.h>
#include <mach/timer.h>

/* PWC register parameter */
#define RDET_EN			0x01

#define WDT_MIN_TIMEOUT	1
#define WDT_MAX_TIMEOUT	4294
#define DEFAULT_TIMEOUT 32	/* 32 sec default timeout */

/* module parameters */
static int timeout = DEFAULT_TIMEOUT;
module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds. "
	"(1<timeout<4294, default=" __MODULE_STRING(DEFAULT_TIMEOUT) ")");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
	"(default=CONFIG_WATCHDOG_NOWAYOUT)");

struct emxx_wdt_func_st {
	int (*set_timeout)(unsigned int);
	void (*ping)(void);
	void (*enable)(void);
	void (*disable)(void);
	void (*setup)(void);
};

static struct emxx_wdt_func_st emxx_wdt_func;
static unsigned long opened;
static char expect_close;

static int emxx_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &opened))
		return -EBUSY;

	emxx_wdt_func.enable();

	return nonseekable_open(inode, file);
}

static int emxx_wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *	Shut off the timer.
	 * 	Lock it in if it's a module and we set nowayout
	 */
	if (expect_close == 42) {
		emxx_wdt_func.disable();
	} else {
		printk(KERN_CRIT "unexpected close, not stopping watchdog!\n");
		emxx_wdt_func.ping();
	}

	clear_bit(0, &opened);
	expect_close = 0;

	return 0;
}

static ssize_t
emxx_wdt_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
	/* Refresh the timer. */
	if (len) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			expect_close = 0;

			/*
			 * scan to see whether or not we got the magic
			 * character
			 */
			for (i = 0; i != len; i++) {
				char c;
				if (get_user(c, data+i))
					return -EFAULT;
				if (c == 'V')
					expect_close = 42;
			}
		}
		emxx_wdt_func.ping();
	}
	return len;
}

static int
emxx_wdt_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	static struct watchdog_info ident = {
		.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT
				| WDIOF_MAGICCLOSE,
		.firmware_version = 0,
		.identity = "emxx Watchdog",
	};
	int new_margin, new_options;
	int ret;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &ident, sizeof(ident)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
			return put_user(0, p);

	case WDIOC_KEEPALIVE:
		emxx_wdt_func.ping();
		return 0;

	case WDIOC_SETOPTIONS:
		ret = -EINVAL;
		if (get_user(new_options, p))
			return -EFAULT;

		if (new_options & WDIOS_DISABLECARD) {
			emxx_wdt_func.disable();
			ret = 0;
		}

		if (new_options & WDIOS_ENABLECARD) {
			emxx_wdt_func.enable();
			ret = 0;
		}

		return ret;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, p))
			return -EFAULT;

		timeout = new_margin;
		if ((new_margin < WDT_MIN_TIMEOUT)
		    || (new_margin > WDT_MAX_TIMEOUT)) {
			timeout = DEFAULT_TIMEOUT;
		}
		emxx_wdt_func.set_timeout(timeout * USEC_PER_SEC);
		/* Fall */
	case WDIOC_GETTIMEOUT:
		return put_user(timeout, p);

	default:
		return -ENOTTY;
	}
}

static int
emxx_notify_sys(struct notifier_block *this, unsigned long code, void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT) {
		/* Turn the WDT off */
		emxx_wdt_func.disable();
	}

	return NOTIFY_DONE;
}

static const struct file_operations emxx_wdt_fops = {
	.owner   = THIS_MODULE,
	.llseek  = no_llseek,
	.ioctl   = emxx_wdt_ioctl,
	.write   = emxx_wdt_write,
	.open    = emxx_wdt_open,
	.release = emxx_wdt_release,
};

static struct miscdevice emxx_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name  = "watchdog",
	.fops  = &emxx_wdt_fops,
};

static struct notifier_block emxx_notifier = {
	.notifier_call = emxx_notify_sys,
};


static int __init emxx_wdt_init(void)
{
	int ret;

	printk(KERN_INFO "Starting wdt.\n");

	ret = misc_register(&emxx_wdt_miscdev);
	if (ret != 0)
		return ret;

	ret = register_reboot_notifier(&emxx_notifier);
	if (ret != 0) {
		printk(KERN_ERR "cannot register reboot notifier (err=%d)\n",
				ret);
		misc_deregister(&emxx_wdt_miscdev);
		return ret;
	}

#ifdef CONFIG_ARCH_EMXX
	emxx_wdt_func.set_timeout = emxx_wdt_set_timeout;
	emxx_wdt_func.ping        = emxx_wdt_ping;
	emxx_wdt_func.enable      = emxx_wdt_enable;
	emxx_wdt_func.disable     = emxx_wdt_disable;
	emxx_wdt_func.setup       = emxx_wdt_setup;
#endif
	/* Init WatchDog timer */
	emxx_wdt_func.setup();
	emxx_wdt_func.set_timeout(timeout * USEC_PER_SEC);

	return 0;
}

static void __exit emxx_wdt_exit(void)
{
	emxx_wdt_func.disable();

	unregister_reboot_notifier(&emxx_notifier);
	misc_deregister(&emxx_wdt_miscdev);
}

module_init(emxx_wdt_init);
module_exit(emxx_wdt_exit);

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("EMMA Mobile series Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
