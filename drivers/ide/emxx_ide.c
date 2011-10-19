/*
 *  File Name		: linux/drivers/ide/emxx_ide.c
 *  Function		: IDE host driver for emxx (EMMA Mobile evolution)
 *                        Static Memory Controller with Compact Flash Card
 *  Release Version : Ver 1.08
 *  Release Date	: 2010/09/01
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *  This program is free software;you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by Free
 *  Softwere Foundation; either version 2 of License, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warrnty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; If not, write to the Free Software Foundation, Inc., 59
 *  Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/ide.h>

#include <asm/mach/irq.h>
#include <mach/dma.h>
#include <mach/gpio.h>
#include <mach/hardware.h>

#include <mach/smu.h>
#include <mach/pmu.h>
#include <mach/pwc.h>

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#include <mach/pm.h>
#endif


#include"emxx_ide.h"

#define DRV_NAME "emxx_cfi_ide"

/* #define CFI_DEBUG */


#define perr(fmt, args...) pr_err(DRV_NAME ": " fmt, ##args)

#ifdef CFI_DEBUG
#define pdbg(fmt, args...) pr_err(DRV_NAME ": %s " fmt, __func__, ##args)
#else
#define pdbg(fmt, args...) pr_debug("%s " fmt, __func__, ##args)
#endif

static struct ide_host *emxx_cfi_host;
static hw_regs_t emxx_cfi_hw;

static volatile int emxx_cfi_connect;
static volatile int emxx_cfi_burst_mode;
static volatile int emxx_cfi_push;

static struct workqueue_struct	*emxx_cfi_workqueue;
static struct delayed_work	emxx_cfi_detect_work;


/*
 * init and config the hardware pin
 */
static void emxx_cfi_init_pin(void)
{
	u32 val;

	/* SMU switch to CFI function */
	val = readl(CHG_PINSEL_AB) & 0xFFFFF000;
	writel(val | 0x00000AAA, CHG_PINSEL_AB);

	writel(readl(CHG_PINSEL_G096) | 0x00000060, CHG_PINSEL_G096);
	gpio_direction_input(GPIO_P101);

	/* GPIO71-75 :PD/PU disable,
	   [71-73 input disable][74:input enable] */
	writel((readl(CHG_PULL7)&0x00000FFF) | 0x40000000, CHG_PULL7);
	/* GPIO76-83,
	   [76: input disable, PD/PU disable][D0-D6:input enable, PD] */
	writel(0x77777770, CHG_PULL8);
	/* GPIO84-91(D7-D14), input enable, pull down */
	writel(0x77777777, CHG_PULL9);
	/* GPIO92-99 */
	writel(0x04443337, CHG_PULL10);
	/* GPIO100-102, input enable, disable PU/PD */
	val = readl(CHG_PULL11) & 0xfffff000;
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1)
		writel(val | 0x00000444, CHG_PULL11);
	else
#endif /* CONFIG_MACH_EMEV */
		writel(val | 0x00000554, CHG_PULL11);

}

/*
 * Init CFI module
 */
static void emxx_cfi_enable(void)
{
	u32 val;

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1) {
#endif /* CONFIG_MACH_EMEV */
		emxx_reset_device(EMXX_RST_CFI);
		emxx_close_clockgate(EMXX_CLK_CFI | EMXX_CLK_CFI_H);
		/* card power on */
		pwc_reg_write(DA9052_LDO8_REG, 0x6A);
		udelay(300);
#ifdef CONFIG_MACH_EMEV
	}
#endif /* CONFIG_MACH_EMEV */

	emxx_open_clockgate(EMXX_CLK_CFI | EMXX_CLK_CFI_H);
	emxx_unreset_device(EMXX_RST_CFI);

	/* config the CFI work in the PIO mode,  hardware reset, 16 bit data */
	writel(CFI_CONTROL_0_IDE, EMXX_CFI_CONTROL_0);

	/* hardware reset release in CFI module */
	val = readl(EMXX_CFI_CONTROL_0) | CFI_CONTROL_0_HRST;
	writel(val, EMXX_CFI_CONTROL_0);

	/* disable all the interrupts */
	writel(0x00000000, EMXX_CFI_CONTROL_1);

	/* clear all the interrupt */
	writel(0xffffffff, EMXX_CFI_INTERRUPT);

	/* PIO mode access timing confige */
	writel(0x00001319, EMXX_CFI_TIMING_1);

	/* setting for PIO */
	val = readl(EMXX_CFI_BUSIF_CTRL) & 0xFFFFFF00;
	writel(val | 0x00000062 | emxx_cfi_burst_mode, EMXX_CFI_BUSIF_CTRL);
}

/*
 * DeInit CFI module
 */
static void emxx_cfi_disable(void)
{
	emxx_reset_device(EMXX_RST_CFI);
	emxx_close_clockgate(EMXX_CLK_CFI | EMXX_CLK_CFI_H);

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1) {
#endif /* CONFIG_MACH_EMEV */
		/* card power off */
		pwc_reg_write(DA9052_LDO8_REG, 0x00);
		mdelay(2);
#ifdef CONFIG_MACH_EMEV
	}
#endif /* CONFIG_MACH_EMEV */
}


/****************************************
 * Defined member of emxx_ide_tp_ops
 ****************************************/
/*
 * Read status function for emxx_ide
 */
static u8 emxx_ide_read_status(ide_hwif_t *hwif)
{
	u8 ret;

	ret = ATA_ERR | ATA_DF;
	if (emxx_cfi_connect)
		ret = ide_read_status(hwif);

	return ret;
}

/*
 * Read status function for emxx_ide
 */
static u8 emxx_ide_read_altstatus(ide_hwif_t *hwif)
{
	u8 ret;

	ret = ATA_ERR | ATA_DF;
	if (emxx_cfi_connect)
		ret = ide_read_altstatus(hwif);

	return ret;
}

/*
 * interrupt control function for emxx_ide
 */
static void emxx_ide_set_irq(ide_hwif_t *hwif, int on)
{
	/*disable all the interrupts*/
	writel(0x00000000, EMXX_CFI_CONTROL_1);

	/*clear all the interrupt*/
	writel(0xffffffff, EMXX_CFI_INTERRUPT);

	if (on)
		writel(CFI_CONTROL1_RINTE, EMXX_CFI_CONTROL_1);

	ide_set_irq(hwif, on);
}

/*
 * tf read function for emxx_ide
 */
static void emxx_ide_tf_read(ide_drive_t *drive, ide_task_t *task)
{
	struct ide_taskfile *tf = &task->tf;

	if (emxx_cfi_connect)
		ide_tf_read(drive, task);
	else {
		tf->error = ATA_UNC;
		tf->status = (ATA_ERR | ATA_DF);
	}
}

/*
 * tf load function for emxx_ide
 */
static void emxx_ide_tf_load(ide_drive_t *drive, ide_task_t *task)
{
	if (emxx_cfi_connect)
		ide_tf_load(drive, task);
}

/*
 * Input data function for emxx_ide
 */
static void emxx_ide_input_data(ide_drive_t *drive, struct request *rq,
				void *buf, unsigned int len)
{
	if (emxx_cfi_connect) {
		emxx_cfi_push++;
		ide_input_data(drive, rq, buf, len);
		emxx_cfi_push--;
	}
}

/*
 * Output data function for emxx_ide
 */
static void emxx_ide_output_data(ide_drive_t *drive, struct request *rq,
				 void *buf, unsigned int len)
{
	if (emxx_cfi_connect) {
		emxx_cfi_push++;
		ide_output_data(drive, rq, buf, len);
		emxx_cfi_push--;
	}
}


/****************************************
 * Defined member of emxx_ide_port_ops
 ****************************************/
/*
 * set the cfi work in PIO mode for emxx_ide
 */
static void emxx_ide_set_pio_mode(ide_drive_t *drive, const u8 pio)
{
	printk("PIO mode = 0x%x\n", pio);

	/* PIO mode access timing confige */
	switch (pio) {
	case 0:
		writel(0x00001319, EMXX_CFI_TIMING_1);
		break;
	case 1:
		writel(0x0000020F, EMXX_CFI_TIMING_1);
		break;
	case 2:
		writel(0x0000010A, EMXX_CFI_TIMING_1);
		break;
	case 3:
		writel(0x00003107, EMXX_CFI_TIMING_1);
		break;
	case 4:
		writel(0x00001106, EMXX_CFI_TIMING_1);
		break;
	case 5:
		writel(0x00001005, EMXX_CFI_TIMING_1);
		break;
	case 6:
		writel(0x00001005, EMXX_CFI_TIMING_1);
		break;
	default:
		perr("PIO mode not support\n");
		break;
	}
}


/****************************************
 * Defined member of emxx_ide_port_info
 ****************************************/
/*
 * ide_tp_ops for emxx_ide 
 */
static const struct ide_tp_ops emxx_ide_tp_ops = {
	.exec_command	= ide_exec_command,
	.read_status	= emxx_ide_read_status,
	.read_altstatus	= emxx_ide_read_altstatus,
	.set_irq	= emxx_ide_set_irq,

	.tf_load	= emxx_ide_tf_load,
	.tf_read	= emxx_ide_tf_read,

	.input_data	= emxx_ide_input_data,
	.output_data	= emxx_ide_output_data,
};

/*
 * ide_port_ops for emxx_ide 
 */
static const struct ide_port_ops emxx_ide_port_ops = {
	.set_pio_mode	= emxx_ide_set_pio_mode,
};

/*
 * ide_port_info for emxx_ide 
 */
static const struct ide_port_info emxx_ide_port_info = {
	.tp_ops		= &emxx_ide_tp_ops,
	.port_ops	= &emxx_ide_port_ops,

	.host_flags 	=  IDE_HFLAG_MMIO
			 | IDE_HFLAG_NO_DMA
			 | IDE_HFLAG_SINGLE
			 | IDE_HFLAG_NO_IO_32BIT
			 | IDE_HFLAG_UNMASK_IRQS,

	.pio_mask 	= ATA_PIO6,
};

/*
 *  interrupt handler function
 */
static irqreturn_t emxx_cfi_irq_handler(int irq, void *dev_id)
{
	int status, int_status;

	udelay(3);

	int_status = readl(EMXX_CFI_INTERRUPT);
	status = readl(EMXX_CFI_STATUS);

	pdbg("irq=0x%x status=0x%x\n", int_status, status);

	if (int_status) {
		writel(int_status, EMXX_CFI_INTERRUPT);
		if ((int_status & CFI_INT_RDYS))
			return ide_intr(irq, dev_id);
	}

	return IRQ_HANDLED;
}

/*
 *  interrupt handler function from BPIO_P101
 */
static irqreturn_t emxx_cfi_detect(int irq, void *dev_id)
{
	if (gpio_get_value(GPIO_P101) != 0) {
		if (emxx_cfi_host)
			emxx_cfi_connect = 0;
	}
	queue_delayed_work(emxx_cfi_workqueue,
			&emxx_cfi_detect_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

/*
 *  CF card detect check function
 */
static void emxx_cfi_detect_check(struct work_struct *work)
{
	hw_regs_t *hws[] = { &emxx_cfi_hw, NULL, NULL, NULL };
	int ret;

	if (gpio_get_value(GPIO_P101) == 0) {
		printk(KERN_INFO "emxx_ide: plag-IN CFCARD\n");
		if (emxx_cfi_host) {
			emxx_cfi_connect = 0;
			mdelay(200);
			ide_host_remove(emxx_cfi_host);
			emxx_cfi_disable();
			emxx_cfi_host = NULL;
		} else {
			emxx_cfi_disable();
			mdelay(200);
		}

		emxx_cfi_host = ide_host_alloc(&emxx_ide_port_info, hws);
		if (!emxx_cfi_host) {
			perr("failed to allocate ide host\n");
			return;
		}

		/* Set the irq handler */
		emxx_cfi_host->irq_handler = emxx_cfi_irq_handler;

		emxx_cfi_enable();

		emxx_cfi_connect = 1;

		emxx_cfi_push++;

		/* Register the CF info into the ide host */
		ret = ide_host_register(emxx_cfi_host,
				&emxx_ide_port_info, hws);

		emxx_cfi_push--;

		if (ret) {
			perr("failed to register ide host\n");
			emxx_cfi_connect = 0;
			ide_host_free(emxx_cfi_host);
			emxx_cfi_disable();
			emxx_cfi_host = NULL;
		}
	} else {
		printk(KERN_INFO "emxx_ide: plag-OUT CFCARD\n");
		if (emxx_cfi_host) {
			emxx_cfi_connect = 0;
			mdelay(200);
			ide_host_remove(emxx_cfi_host);
			emxx_cfi_disable();
			emxx_cfi_host = NULL;
		}
	}
}

/*
 * suspend function for emxx_ide
 */
static int emxx_cfi_suspend(struct platform_device *pdev, pm_message_t state)
{
	switch (state.event) {
	case PM_EVENT_SUSPEND:
		if (emxx_cfi_push)
			return -EBUSY;
		break;
	default:
		break;
	}
	return 0;
}

/*
 * resume function for emxx_ide
 */
static int emxx_cfi_resume(struct platform_device *dev)
{
	return 0;
}

/*
 * probe function, mainly used to get the taskfile base address and
 * configure the hardware information for the IDE framework
 */
static int __init emxx_cfi_probe(struct platform_device *pdev)
{
	int ret;

	emxx_cfi_push = 0;

	emxx_cfi_burst_mode = CFI_BURST_MODE_3 << 3;
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1)
		emxx_cfi_burst_mode = CFI_BURST_MODE_0 << 3;
#endif /* CONFIG_MACH_EMEV */

	memset(&emxx_cfi_hw, 0, sizeof(emxx_cfi_hw));
	ide_std_init_ports(&emxx_cfi_hw,
		EMXX_CFI_TASK_FILE, EMXX_CFI_CTL_MODE + 6);

	emxx_cfi_hw.irq = INT_CFI;
	emxx_cfi_hw.dev =  &pdev->dev;
	emxx_cfi_hw.chipset = ide_generic;

	/* Setting pin */
	emxx_cfi_init_pin();

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif /* CONFIG_MACH_EMEV */
		emxx_cfi_disable();

	emxx_cfi_workqueue = create_singlethread_workqueue(DRV_NAME);
	INIT_DELAYED_WORK(&emxx_cfi_detect_work,
				emxx_cfi_detect_check);

	set_irq_type(INT_GPIO_101, IRQ_TYPE_EDGE_BOTH);
	ret = request_irq(INT_GPIO_101, emxx_cfi_detect,
				IRQF_DISABLED, DRV_NAME, NULL);

	queue_delayed_work(emxx_cfi_workqueue,
			&emxx_cfi_detect_work, msecs_to_jiffies(100));

	return 0;
}

/*
 * remove function, release the driver resource
 */
static int __exit emxx_cfi_remove(struct platform_device *pdev)
{
	if (emxx_cfi_host) {
		ide_host_remove(emxx_cfi_host);
		emxx_cfi_disable();
	}

	free_irq(INT_GPIO_101, NULL);
	destroy_workqueue(emxx_cfi_workqueue);

	return 0;
}

static struct platform_driver emxx_cfi_driver = {
	.probe	= emxx_cfi_probe,
	.remove	= __exit_p(emxx_cfi_remove),

	.suspend	= emxx_cfi_suspend,
	.resume		= emxx_cfi_resume,

	.driver	= {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init emxx_cfi_init(void)
{
	return platform_driver_register(&emxx_cfi_driver);
}

static void __exit emxx_cfi_exit(void)
{
	platform_driver_unregister(&emxx_cfi_driver);
}

module_init(emxx_cfi_init);
module_exit(emxx_cfi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Renesas Electronics Corporation");

