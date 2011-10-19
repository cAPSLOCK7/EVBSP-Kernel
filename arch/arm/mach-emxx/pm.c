/*
 *  File Name       : arch/arm/mach-emxx/pm.c
 *  Function        : emxx_board
 *  Release Version : Ver 1.12
 *  Release Date    : 2011/01/24
 *
 * Copyright (C) 2010-2011 Renesas Electronics Corporation
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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/cpu.h>

#include <asm/irq.h>
#include <asm/mach/time.h>
#include <asm/cpu.h>

#include <mach/system.h>
#include <mach/smu.h>
#include <mach/pm.h>
#include <mach/pmu.h>

int emxx_sleep_while_idle;
EXPORT_SYMBOL(emxx_sleep_while_idle);

#ifdef CONFIG_PM
#include <linux/cpu.h>
#include <linux/suspend.h>
#include <mach/dma.h>
#include "pm_pmu.h"

static unsigned long pm_loops_per_jiffy;
static int sleep_while_idle_enable;
static unsigned int sleep_while_idle_count;
static unsigned int pm_idle_count;
static unsigned int skip_ktimer_idle_count;
static unsigned int suspend_disable;
static int emxx_can_sleep(void);
static int emxx_pm_enter(suspend_state_t state);
static int emxx_pm_valid(suspend_state_t state);
static void emxx_pm_finish(void);
static void variable_init(void);
static int emxx_pm_suspend(suspend_state_t state);
static DEFINE_SPINLOCK(pdma_ena_lock);

static void variable_init(void)
{
	/* initialize static variable */
	emxx_sleep_while_idle = 0;
	pm_loops_per_jiffy = 0;
	sleep_while_idle_enable = 0;
	sleep_while_idle_count = 0;
	pm_idle_count = 0;
	skip_ktimer_idle_count = 0;
	suspend_disable = 0;
}

/* refered by emxx_target() */
void emxx_change_normalx(unsigned int normal_div)
{
	ulong auto_frq_change;

	if ((inl(SMU_CLK_MODE_SEL) & 0x00000F00) == (normal_div << 8))
		return;

	/* auto freq change off */
	auto_frq_change = inl(SMU_CKRQ_MODE);
	outl((auto_frq_change & ~0x1), SMU_CKRQ_MODE);

	outl(normal_div, SMU_CLK_MODE_SEL);
	while ((inl(SMU_CLK_MODE_SEL) & 0x00000F00) != (normal_div << 8))
		;

	/* restore auto freq change */
	outl(auto_frq_change, SMU_CKRQ_MODE);
}

void emxx_pm_idle(void)
{
	local_irq_disable();
	local_fiq_disable();
	if (need_resched()) {
		local_fiq_enable();
		local_irq_enable();
		return;
	}

	pm_idle_count++;

	if (sleep_while_idle_enable && !suspend_disable) {
#ifdef CONFIG_SMP
		if ((0 == hard_smp_processor_id()) &&
			!(inl(SMU_CLKSTOPSIG_ST) & PWSTATE_PD_NE1)) {
			/* cpu1 wfi */
			emxx_close_clockgate(EMXX_CLK_TI1);
#endif
			if (emxx_can_sleep()) {
				emxx_sleep_while_idle = 1;
				if (0 == emxx_pm_suspend(PM_SUSPEND_STANDBY)) {
					emxx_add_neigh_timer();
					emxx_add_timer_writeback();
					emxx_add_workqueue_timer();

					emxx_sleep_while_idle = 0;
					sleep_while_idle_count++;
#ifdef CONFIG_SMP
					emxx_open_clockgate(EMXX_CLK_TI1);
#endif
					set_need_resched();

					local_fiq_enable();
					local_irq_enable();

					return;
				}
				emxx_add_neigh_timer();
				emxx_add_timer_writeback();
				emxx_add_workqueue_timer();

				emxx_sleep_while_idle = 0;
			}
#ifdef CONFIG_SMP
			emxx_open_clockgate(EMXX_CLK_TI1);
		}
#endif
	}

	arch_idle();

	local_fiq_enable();
	local_irq_enable();
}


static int check_transfer_state(void)
{
	unsigned int state;

#ifdef CONFIG_SMP
	if (inl(SMU_CLKSTOPSIG_ST) & PWSTATE_PD_NE1)
		return PM_ERROR_CPU1;
#endif
	if (readl(INTD_IT3_RAW0) || readl(INTD_IT3_IEN1)
#ifdef CONFIG_MACH_EMGR
		|| (readl(INTD_IT3_RAW2) & INTD_LIIS01)
#endif
		|| !(readl(SMU_POWER_STATUS) & PD_PWRSTATUS))
		return PM_ERROR_DSP;

	if (inl(IMC_STATUS) || inl(IMCW_STATUS))
		return PM_ERROR_IMC;

	/* siz: if working off(0) */
	if (!(inl(SMU_AHBCLKCTRL1) & AHBSIZ_CHECK_BIT))
		return PM_ERROR_SIZ;

#ifdef CONFIG_KEYBOARD_EMXX_MAX7318
	if (emxx_key_pm_state() != 0)
		return PM_ERROR_KEY;
#endif
	if (inl(SMU_PWMGCLKCTRL) & GCLKPWM_CHECK_BIT)
		return PM_ERROR_PWM;
#ifdef CONFIG_MMC_EMXX_SDIO
	if (inl(SMU_SDIO0GCLKCTRL) & GCLKSDIO_CHECK_BIT)
		return PM_ERROR_SDIO0;
	if (inl(SMU_SDIO1GCLKCTRL) & GCLKSDIO_CHECK_BIT)
		return PM_ERROR_SDIO1;
#ifdef CONFIG_MACH_EMEV
	if (inl(SMU_SDIO2GCLKCTRL) & GCLKSDIO_CHECK_BIT)
		return PM_ERROR_SDIO2;
#endif
#endif
#ifdef CONFIG_MMC_EMXX_SDC
	if (inl(SMU_SDCGCLKCTRL) & GCLKSDIO_CHECK_BIT)
		return PM_ERROR_SDC;
#endif

	/* USB host */
	/* 1: not connecting */
	/* 0: connecting */
	if (!(inl(SMU_CKRQMODE_MASK1) & GCLKUSB_CHECK_BIT))
		return PM_ERROR_USBH;

	/* check usb func */
	/* 1: VBUS ON */
	/* 0: VBUS OFF */
	if (gpio_get_value(GPIO_VBUS))
		return PM_ERROR_USBF;

#ifdef CONFIG_TOUCHSCREEN_DA9052
	if (emxx_touch_pm_state() != 0)
		return PM_ERROR_TOUCH;
#endif
	/* SPI */
	if (inl(SMU_USIBS2GCLKCTRL) & GCLKSPI_CHECK_BIT)
		return PM_ERROR_SPI;

	/* NTS */
	if (inl(SMU_NTSGCLKCTRL) & GCLKNTS_CHECK_BIT)
		return PM_ERROR_NTS;

	/* AVE */
	if (inl(SMU_AVEGCLKCTRL) & GCLKAVE_CHECK_BIT)
		return PM_ERROR_AVE;

	if (inl(SMU_IIC0GCLKCTRL) & GCLKIIC_CHECK_BIT)
		return PM_ERROR_IIC0;

	if (inl(SMU_IIC1GCLKCTRL) & GCLKIIC_CHECK_BIT)
		return PM_ERROR_IIC1;

#ifdef CONFIG_MACH_EMGR
	/* A2D */
	if (inl(SMU_A2DGCLKCTRL) & GCLKA2D_CHECK_BIT)
		return PM_ERROR_A2D;
#endif
#ifdef CONFIG_MACH_EMEV
	/* A3D */
	if (inl(SMU_A3DGCLKCTRL) & GCLKA3D_CHECK_BIT)
		return PM_ERROR_A3D;
#endif
	if (emxx_dma_busy())
		return PM_ERROR_DMA;


	state = inl(LCD_STATUS);
	/* display on, and DirectPath(BIT10-8 is '001')*/
	if (state & 0x01) {
#ifdef CONFIG_MACH_EMEV
		if ((state & 0x00000700) != 0x00000100)
			return PM_ERROR_LCD;
		/* Through here at DirectPath */
#else
		return PM_ERROR_LCD;
#endif
	}
	return 0;
}


static int emxx_can_sleep(void)
{
	unsigned long last_jiffies, delta_jiffies;
	int ret = 0;

	if (!mutex_trylock(&pm_mutex))
		return 0;

	if (check_transfer_state())
		goto Unlock;

	emxx_del_neigh_timer();
	emxx_del_timer_writeback();
	emxx_del_workqueue_timer();

	last_jiffies = jiffies;
	delta_jiffies = get_next_timer_interrupt(last_jiffies) - last_jiffies;
	if (delta_jiffies <= EMXX_SLEEP_THRESHOLD) {
		emxx_add_neigh_timer();
		emxx_add_timer_writeback();
		emxx_add_workqueue_timer();
		skip_ktimer_idle_count++;
		goto Unlock;
	}

	/* go sleep */
	ret = 1;

Unlock:
	mutex_unlock(&pm_mutex);
	return ret;
}


static ssize_t sleep_while_idle_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %u %u %u\n",
		sleep_while_idle_enable,
		sleep_while_idle_count,
		skip_ktimer_idle_count,
		pm_idle_count
		);
}

static ssize_t sleep_while_idle_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t n)
{
	int value;
	if (sscanf(buf, "%d", &value) != 1 ||
	    (value != 0 && value != 1)) {
		printk(KERN_ERR "idle_sleep_store: Invalid value\n");
		return -EINVAL;
	}
	if (value)
		emxx_open_clockgate(EMXX_CLK_TI2);
	else
		emxx_close_clockgate(EMXX_CLK_TI2);

	sleep_while_idle_enable = value;
	return n;
}

static struct kobj_attribute sleep_while_idle_attr =
	__ATTR(sleep_while_idle, 0644, sleep_while_idle_show,
						sleep_while_idle_store);



/* this table's value = div*2 */
static int div_table[12] = {
	2, 4, 6, 8, 12, 16, 24, 32, 3, 5, 10, 20
};

/*
 return value:
   "div*2"
*/
static int pm_get_acpu_div(int div_id)
{
	ulong div;

	switch (div_id) {
	case NORMAL_A:
		div = (inl(SMU_NORMALA_DIV) & 0x0F);
		break;
	case NORMAL_B:
		div = (inl(SMU_NORMALB_DIV) & 0x0F);
		break;
	case NORMAL_C:
		div = (inl(SMU_NORMALC_DIV) & 0x0F);
		break;
	case NORMAL_D:
	default:
		div = (inl(SMU_NORMALD_DIV) & 0x0F);
		break;
	}

	if (div < 12)
		div = div_table[div];
	else
		div = 2;

	return div;
}

#ifdef CONFIG_SMP
static inline void pm_smp_change_jiffies(void)
{
	int i;

	for_each_online_cpu(i)
		per_cpu(cpu_data, i).loops_per_jiffy = loops_per_jiffy;
}
#endif

void pm_change_normalA(void)
{
	emxx_change_normalx(NORMAL_A);

	loops_per_jiffy = pm_loops_per_jiffy;
#ifdef CONFIG_SMP
	pm_smp_change_jiffies();
#endif
}

void pm_change_normalB(void)
{
	/* set clock to div4(133MHz) of Mode A */
	outl(NORMAL_B_VALUE, SMU_NORMALB_DIV);		/* 133MHz */
	emxx_change_normalx(NORMAL_B);

	/* div*2 */
	loops_per_jiffy = pm_loops_per_jiffy/(pm_get_acpu_div(NORMAL_B)/2);
#ifdef CONFIG_SMP
	pm_smp_change_jiffies();
#endif

}

static int emxx_pm_suspend(suspend_state_t state)
{
	unsigned int sleep_mode;

	switch (state) {
	case PM_SUSPEND_STANDBY:
		state = inl(LCD_STATUS);
		if (state & 0x01) {
#ifdef CONFIG_MACH_EMEV
			/* re-check.. */
			if ((state & 0x00000700) != 0x00000100)
				return -EFAULT;
			sleep_mode = EMXX_PMU_CLK_ECONOMY;
#else
			return -EFAULT;
#endif
		} else
			sleep_mode = EMXX_PMU_CLK_SLEEP;

		break;
	case PM_SUSPEND_MEM:
	default:
		sleep_mode = EMXX_PMU_CLK_DEEPSLEEP;
		break;
	}

	if (emxx_sleep_while_idle) {
		if (serial8250_idle_suspend() != 0) {
			serial8250_idle_resume();
			return -EFAULT;
		}
#ifdef CONFIG_USB_EHCI_HCD
		emxx_ehci_idle_suspend();
#endif
#ifdef CONFIG_USB_OHCI_HCD
		emxx_ohci_idle_suspend();
#endif
#ifdef CONFIG_MMC_EMXX_SDIO
		emxx_sdio_idle_suspend();
#endif
	}

	emxx_pmu_sleep(sleep_mode);

	if (emxx_sleep_while_idle) {
		serial8250_idle_resume();
#ifdef CONFIG_USB_EHCI_HCD
		emxx_ehci_idle_resume();
#endif
#ifdef CONFIG_USB_OHCI_HCD
		emxx_ohci_idle_resume();
#endif
	}

	return 0;
}

static int emxx_pm_valid(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return 1;

	default:
		return 0;
	}
}

static int emxx_pm_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return emxx_pm_suspend(state);

	default:
		return -EINVAL;
	}

	return 0;
}

static void emxx_pm_finish(void)
{
}

static struct platform_suspend_ops emxx_pm_ops = {
	.valid = emxx_pm_valid,
	.enter = emxx_pm_enter,
	.finish = emxx_pm_finish,
};

void emxx_pm_power_off(void)
{
	unsigned long irq_flags;

	local_irq_save(irq_flags);

	/* poweroff setting function*/
	emxx_pm_do_poweroff();

	printk(KERN_INFO "%s(): AFTER PMU POWER OFF.\n", __func__);

	local_irq_restore(irq_flags);	  /* not call */
}


void emxx_pm_pdma_suspend_enable(void)
{
	unsigned long flags;

	spin_lock_irqsave(&pdma_ena_lock, flags);
	suspend_disable--;
	spin_unlock_irqrestore(&pdma_ena_lock, flags);
	udelay(1);
}
EXPORT_SYMBOL(emxx_pm_pdma_suspend_enable);

void emxx_pm_pdma_suspend_disable(void)
{
	unsigned long flags;

	spin_lock_irqsave(&pdma_ena_lock, flags);
	suspend_disable++;
	spin_unlock_irqrestore(&pdma_ena_lock, flags);
}
EXPORT_SYMBOL(emxx_pm_pdma_suspend_disable);

unsigned int emxx_pm_pdma_suspend_status(void)
{
	return suspend_disable;
}
EXPORT_SYMBOL(emxx_pm_pdma_suspend_status);


#ifdef CONFIG_PROC_FS
/*
 * Writing to /proc/pm puts the CPU in sleep mode
 */
static ssize_t emxx_pm_write_proc(struct file *file, const char *buffer,
				   size_t count, loff_t *ppos)
{
	char *buf;
	char *tok;
	char *s;
	char *str;
	char *whitespace = " \t\r\n";
	int ret = 0;

	if (current_uid() != 0)
		return -EPERM;
	if (count == 0)
		return 0;

	buf = kmalloc(count + 1, GFP_KERNEL);
	if (buf == NULL) {
		ret = -ENOMEM;
		goto exit_2;
	}
	if (copy_from_user(buf, buffer, count) != 0) {
		ret = -EFAULT;
		goto exit_1;
	}
	buf[count] = '\0';
	s = buf + strspn(buf, whitespace);
	tok = strsep(&s, whitespace);

	str = "HIGHCLK";
	if (strnicmp(tok, str, strlen(str) + 1) == 0) {
		printk(KERN_INFO "533MHz...\n");
		pm_change_normalA();
		goto exit_1;
	}
	str = "LOWCLK";
	if (strnicmp(tok, str, strlen(str) + 1) == 0) {
		printk(KERN_INFO "133MHz...\n");
		pm_change_normalB();
		goto exit_1;
	}

	ret = -EINVAL;

exit_1:
	kfree(buf);
exit_2:
	if (ret == 0)
		return count;

	return ret;
}

static int emxx_pm_read_proc(char *page, char **start, off_t off, int count,
			      int *eof, void *data)
{
	return 0;
}
#endif	/* CONFIG_PROC_FS */


int __init emxx_pm_init(void)
{
	int ret;

	printk(KERN_INFO "Power Management for EMXX.\n");

	/* initialize static variable */
	variable_init();

	pm_idle = emxx_pm_idle;
	suspend_set_ops(&emxx_pm_ops);
	pm_loops_per_jiffy = loops_per_jiffy;

	ret = sysfs_create_file(power_kobj, &sleep_while_idle_attr.attr);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

#ifdef CONFIG_PROC_FS
	{
		struct proc_dir_entry *entry;
		entry = create_proc_read_entry("pm", S_IWUSR | S_IRUGO, NULL,
					       emxx_pm_read_proc, 0);
		if (entry == NULL)
			return -ENOENT;
		entry->write_proc = (write_proc_t *)emxx_pm_write_proc;
	}
#endif

	pm_power_off = emxx_pm_power_off;

	return 0;
}

device_initcall(emxx_pm_init);
#endif
