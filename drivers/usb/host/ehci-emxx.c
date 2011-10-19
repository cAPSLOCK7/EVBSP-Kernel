/*
 *  EHCI HCD (Host Controller Driver) for USB.
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

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/signal.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>

#include <mach/hardware.h>
#include <mach/pmu.h>
#include <mach/smu.h>
#include <mach/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include "ehci-emxx.h"


/* ----- Work ----- */
struct emxx_hcd_work	*pg_hcd_work;


#ifdef CONFIG_USB_EHCI_TEST_MODE
	#include "ehci-emxx_test.c"
#endif	/* CONFIG_USB_EHCI_TEST_MODE */


static int _emxx_hc_start(struct usb_hcd *hcd);
static void _emxx_hc_stop(struct usb_hcd *hcd);
static void _emxx_vbus_control(int on_flag);





/*-------------------------------------------------------------------------*/
/*
 * Save OHCI Register
*/
static void _emxx_ohci_save_register(void)
{
	struct emxx_ohci_reg *p_reg;

	p_reg = &pg_hcd_work->ohci_reg;
	if (p_reg->save_flag)
		return;

	p_reg->save_flag = 1;

	p_reg->HcInterruptEnable = emxx_io_read32(EMXX_OHCI_HCINTERRUPTENABLE);
	p_reg->HcControl = emxx_io_read32(EMXX_OHCI_HCCONTROL);
	p_reg->HcCommandStatus = emxx_io_read32(EMXX_OHCI_HCCOMMANDSTATUS);
	p_reg->HcHCCA = emxx_io_read32(EMXX_OHCI_HCHCCA);
	p_reg->HcControlHeadED = emxx_io_read32(EMXX_OHCI_HCCONTROLHEADED);
	p_reg->HcControlCurrentED =
		emxx_io_read32(EMXX_OHCI_HCCONTROLCURRENTED);
	p_reg->HcBulkHeadED = emxx_io_read32(EMXX_OHCI_HCBULKHEADED);
	p_reg->HcBulkCurrentED = emxx_io_read32(EMXX_OHCI_HCBULKCURRENTED);
	p_reg->HcFmInterval = emxx_io_read32(EMXX_OHCI_HCFMINTERVAL);
	p_reg->HcPeriodicStart = emxx_io_read32(EMXX_OHCI_HCPERIODICSTART);
	p_reg->HcLSThreshold = emxx_io_read32(EMXX_OHCI_HCLSTHRESHOLD);

	p_reg->HcRhDescrptorA = emxx_io_read32(EMXX_OHCI_HCRHDESCRPTORA);
	p_reg->HcRhDescrptorB = emxx_io_read32(EMXX_OHCI_HCRHDESCRPTORB);
	p_reg->HcRhStatus = emxx_io_read32(EMXX_OHCI_HCRHSTATUS);
	p_reg->HcRhPortStatus = emxx_io_read32(EMXX_OHCI_HCRHPORTSTATUS);
}

/*-------------------------------------------------------------------------*/
/*
 * Restore OHCI Register
*/
static void _emxx_ohci_load_register(void)
{
	struct emxx_ohci_reg *p_reg;

	p_reg = &pg_hcd_work->ohci_reg;
	if (p_reg->save_flag == 0)
		return;

	emxx_io_write32(EMXX_OHCI_HCRHPORTSTATUS, p_reg->HcRhPortStatus);
	emxx_io_write32(EMXX_OHCI_HCRHSTATUS, p_reg->HcRhStatus);
	emxx_io_write32(EMXX_OHCI_HCRHDESCRPTORB, p_reg->HcRhDescrptorB);
	emxx_io_write32(EMXX_OHCI_HCRHDESCRPTORA, p_reg->HcRhDescrptorA);

	emxx_io_write32(EMXX_OHCI_HCLSTHRESHOLD, p_reg->HcLSThreshold);
	emxx_io_write32(EMXX_OHCI_HCPERIODICSTART, p_reg->HcPeriodicStart);
	emxx_io_write32(EMXX_OHCI_HCFMINTERVAL, p_reg->HcFmInterval);
	emxx_io_write32(EMXX_OHCI_HCBULKCURRENTED, p_reg->HcBulkCurrentED);
	emxx_io_write32(EMXX_OHCI_HCBULKHEADED, p_reg->HcBulkHeadED);
	emxx_io_write32(EMXX_OHCI_HCCONTROLCURRENTED,
				p_reg->HcControlCurrentED);
	emxx_io_write32(EMXX_OHCI_HCCONTROLHEADED, p_reg->HcControlHeadED);
	emxx_io_write32(EMXX_OHCI_HCHCCA, p_reg->HcHCCA);
	emxx_io_write32(EMXX_OHCI_HCCOMMANDSTATUS, p_reg->HcCommandStatus);
	emxx_io_write32(EMXX_OHCI_HCCONTROL, p_reg->HcControl);
	emxx_io_write32(EMXX_OHCI_HCINTERRUPTENABLE, p_reg->HcInterruptEnable);

	p_reg->save_flag = 0;
}

/*-------------------------------------------------------------------------*/
/*
 * VBUS ON/OFF
 *
 * on_flag
 *    0    : VBUS OFF
 *  !=0    : VBUS ON
*/
void emxx_hc_vbus_control(int on_flag)
{
	_emxx_vbus_control(on_flag);
}

/*-------------------------------------------------------------------------*/
/*
 * Reset Controller
 *
*/
static void _emxx_reset_controller(struct work_struct *work)
{
	struct emxx_hcd_work *p_work = pg_hcd_work;
	struct usb_hcd *hcd = p_work->p_hcd;

	msleep(3000);		/* VBUS OFF wait */

	if (hcd != NULL) {

		_emxx_ohci_save_register();
		usb_remove_root_hub(hcd);
		msleep(1000);
		_emxx_hc_stop(hcd);

		msleep(100);

		_emxx_hc_start(hcd);
		_emxx_ohci_load_register();
		usb_add_root_hub(hcd, INT_USBH, IRQF_DISABLED | IRQF_SHARED);

	}
}


/*-------------------------------------------------------------------------*/
/*
 * Fatal Recovery
 *
*/
void emxx_hc_fatal_recovery(struct usb_hcd *hcd)
{
	struct emxx_hcd_work *p_work = pg_hcd_work;

	_emxx_vbus_control(0);
	p_work->p_hcd = hcd;

	queue_delayed_work(
		p_work->emxx_restart_workqueue,
		&p_work->emxx_restart_work, msecs_to_jiffies(0));
}


/*-------------------------------------------------------------------------*/
/*
 * initialize
 *   initialize for Work area
*/
static int _emxx_init(struct platform_device *pdev)
{
	int	nret = 0;
	struct emxx_hcd_work	*p_work;

	pg_hcd_work = NULL;	/* global */

	p_work = kzalloc(sizeof(struct emxx_hcd_work), GFP_KERNEL);
	if (p_work != NULL) {
		pg_hcd_work = p_work;

		spin_lock_init(&p_work->lock);
		p_work->dma_mask = DMA_32BIT_MASK;

		/*-----------------------------------------------------------*/
		/* DMA parameter setting */
		pdev->dev.dma_mask = &p_work->dma_mask;
		pdev->dev.coherent_dma_mask = DMA_32BIT_MASK;

#ifdef CONFIG_USB_EHCI_TEST_MODE
		nret = _emxx_test_mode_init(p_work);
#endif	/* CONFIG_USB_EHCI_TEST_MODE */

		p_work->emxx_restart_workqueue =
			create_singlethread_workqueue("ehci-wq");
		INIT_DELAYED_WORK
			(&p_work->emxx_restart_work, _emxx_reset_controller);

	} else {
		nret = -ENOMEM;
	}

	return nret;
}

/*-------------------------------------------------------------------------*/
/*
 * end
 *   Work area memory free
*/
static int _emxx_exit(struct platform_device *pdev)
{
	int		nret = 0;

	if (pg_hcd_work != NULL) {

#ifdef CONFIG_USB_EHCI_TEST_MODE
		_emxx_test_mode_exit(pg_hcd_work);
#endif	/* CONFIG_USB_EHCI_TEST_MODE */

		destroy_workqueue(pg_hcd_work->emxx_restart_workqueue);

		kfree(pg_hcd_work);
		pg_hcd_work = NULL;
	}

	return nret;
}

/*-------------------------------------------------------------------------*/
/*
 *  initialize for AHB, PCI Bridge
*/
static int _ahb_pci_bridge_init(void)
{
	u32		data, tmp;
	struct emxx_hcd_work *p_work = pg_hcd_work;

	if (p_work == NULL)
		return -ENODEV;

	/* Clock & Reset & Direct Power Down */
	data = emxx_io_read32(USBCTR);
	if ((data & USBH_RST) == 0)
		return 0;

	data &= ~(DIRPD);
	emxx_io_write32(USBCTR, data);
#ifdef CONFIG_MACH_EMEV
	data &= ~(PCICLK_MASK | USBH_RST);
#else
	data &= ~(PLL_RST | PCICLK_MASK | USBH_RST);
#endif
	emxx_io_write32(USBCTR, data);

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1)
		data = SMODE_READY_CTR | MMODE_HBUSREQ | MMODE_SINGLE_MODE
			 | MMODE_WR_INCR | MMODE_BYTE_BURST;
	else
#endif
		data = SMODE_READY_CTR | MMODE_HBUSREQ
			 | MMODE_WR_INCR | MMODE_BYTE_BURST | MMODE_HTRANS;

	tmp = emxx_io_read32(AHB_BUS_CTR);
	if (data == tmp)
		return 0;

	/* VBUS GPIO & Over Current Setting */
	gpio_direction_output(USB_VBUS_GPIO, 0);
	gpio_direction_input(USB_OVER_CURRENT);

	/****** AHB-PCI Bridge Communication Registers******/
	/* AHB_BUS_CTR */
	emxx_io_write32(AHB_BUS_CTR, data);

	/* PCIAHB_WIN1_CTR <= 0x40000003 */
	emxx_io_write32(PCIAHB_WIN1_CTR, EMXX_SDRAM_BASE | PREFETCH);

	/* PCIAHB_WIN2_CTR <= 0xF0000003 */
	emxx_io_write32(PCIAHB_WIN2_CTR, EMXX_SRAM_BASE | PREFETCH);

	/* AHBPCI_WIN2_CTR <= 0xe2700006 */
	emxx_io_write32(AHBPCI_WIN2_CTR, OHCI_BASE | PCIWIN2_PCICMD);

	/* PCI_ARBITER_CTR */
	data = emxx_io_read32(PCI_ARBITER_CTR);
	data |= (PCIBP_MODE | PCIREQ1 | PCIREQ0);
	emxx_io_write32(PCI_ARBITER_CTR, data);

	/****** PCI Configuration Registers for AHBPCI ******/
	emxx_io_write32(AHBPCI_WIN1_CTR, PCIWIN1_PCICMD | AHB_CFG_AHBPCI);

	/* BASEAD <= 0xe2710800 */
	emxx_io_write32(BASEAD, AHBPCI_BASE);

	/* WIN1_BASEAD <= 0x40000000 */
	emxx_io_write32(WIN1_BASEAD, EMXX_SDRAM_BASE);

	/* WIN2_BASEAD <= 0xF0000000 */
	emxx_io_write32(WIN2_BASEAD, EMXX_SRAM_BASE);

	emxx_io_write32(CMND_STS, SERREN | PERREN | MASTEREN | MEMEN);

	/****** PCI Configuration Registers for OHCI/EHCI ******/
	emxx_io_write32(AHBPCI_WIN1_CTR, PCIWIN1_PCICMD | AHB_CFG_HOST);

	/* OHCI_BASEAD <= 0xe2700000 */
	emxx_io_write32(OHCI_BASEAD, OHCI_BASE);

	/* OHCI_BASEAD <= 0xe2701000 */
	emxx_io_write32(EHCI_BASEAD, EHCI_BASE);

	emxx_io_write32(OHCI_CMND_STS, SERREN | PERREN | MASTEREN | MEMEN);
	emxx_io_write32(EHCI_CMND_STS, SERREN | PERREN | MASTEREN | MEMEN);

#if defined(CONFIG_MACH_EMGR)
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
#elif defined(CONFIG_MACH_EMEV)
	if ((system_rev & EMXX_REV_MASK) < EMXX_REV_ES3) {
#endif
		/* TRANSCIVER (PHY) Setting */
		data = emxx_io_read32(TRANSCIVER_CHARACTERISTIC);
		data &= ~(PORT1_SQU | PORT1_HSIUP);
		data |= USB_PORT1_SQU | USB_PORT1_HSIUP;
		emxx_io_write32(TRANSCIVER_CHARACTERISTIC, data);
#if defined(CONFIG_MACH_EMGR) || defined(CONFIG_MACH_EMEV)
	}
#endif

	/* PCI_INT_ENABLE */
	data = emxx_io_read32(PCI_INT_ENABLE);
	data |= USBH_PMEEN | USBH_INTBEN | USBH_INTAEN;
	emxx_io_write32(PCI_INT_ENABLE, data);

	return 0;
}

static void _ahb_pci_bridge_exit(void)
{
	u32		data;

	/* Clock & Reset & Direct Power Down */
	data = emxx_io_read32(USBCTR);
	data |= DIRPD | PCICLK_MASK | USBH_RST;
	emxx_io_write32(USBCTR, data);

	/* VBUS GPIO Setting */
	gpio_direction_output(USB_VBUS_GPIO, 0);
}

static int _emxx_hc_start(struct usb_hcd *hcd)
{
	int		nret;
#ifdef CONFIG_MACH_EMGR
	u32		val;

	val = readl(SMU_OSC1CTRL1);
	writel(val & ~0x1, SMU_OSC1CTRL1);
	if (val == 0xff) {
		while ((readl(SMU_PLL_STATUS) & 0x00100000) == 0)
			udelay(10);
	}
	writel(1, SMU_USBPHY_HOST_FUNC_SEL);
#endif

	/* Start Clock */
#ifdef CONFIG_MACH_EMEV
	emxx_open_clockgate(EMXX_CLK_USB0 | EMXX_CLK_USB_PCI);
#elif defined(CONFIG_MACH_EMGR)
	emxx_open_clockgate(EMXX_CLK_USB0 | EMXX_CLK_USB1 | EMXX_CLK_USB_PCI);
#endif

	/* Reset State OFF */
	emxx_unreset_device(EMXX_RST_USB0);
#ifdef CONFIG_MACH_EMGR
	emxx_unreset_device(EMXX_RST_USB1);
	val = readl(IO_ADDRESS(EMXX_USBS1_BASE) + 0x1010) & ~0x1000;
	writel(val, IO_ADDRESS(EMXX_USBS1_BASE) + 0x1010);
	val = readl(IO_ADDRESS(EMXX_USBS1_BASE) + 0x1014) | 0x80000000;
	writel(val, IO_ADDRESS(EMXX_USBS1_BASE) + 0x1014);
#endif
	nret = _ahb_pci_bridge_init();

	return nret;
}

static void _emxx_hc_stop(struct usb_hcd *hcd)
{
	_ahb_pci_bridge_exit();

	/* Reset State ON */
	emxx_reset_device(EMXX_RST_USB0);
#ifdef CONFIG_MACH_EMGR
	emxx_reset_device(EMXX_RST_USB1);
#endif

	/* Stop Clock */
#ifdef CONFIG_MACH_EMEV
	emxx_close_clockgate(EMXX_CLK_USB0 | EMXX_CLK_USB_PCI);
#elif defined(CONFIG_MACH_EMGR)
	emxx_close_clockgate(EMXX_CLK_USB0 | EMXX_CLK_USB1 | EMXX_CLK_USB_PCI);
#endif

#ifdef CONFIG_MACH_EMGR
	writel(0, SMU_USBPHY_HOST_FUNC_SEL);
	writel(readl(SMU_OSC1CTRL1) | 0x1, SMU_OSC1CTRL1);
#endif
}

/*-------------------------------------------------------------------------*/
/*
 * VBUS ON/OFF
 *
 * on_flag
 *    0    : VBUS OFF
 *  !=0    : VBUS ON
*/
static void _emxx_vbus_control(int on_flag)
{
	if (on_flag) {
		pr_debug("VBUS ON\n");
		gpio_set_value(USB_VBUS_GPIO, 1);
	} else {
		pr_debug("VBUS OFF\n");
		gpio_set_value(USB_VBUS_GPIO, 0);
		pg_hcd_work->vbus_flag = 1;
	}
}

/*-------------------------------------------------------------------------*/
/*
 * Root HUB Status Data
*/
static int _emxx_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	int		status;

	status = ehci_hub_status_data(hcd, buf);

	if (status) {
		u32		temp;
		struct ehci_hcd		*ehci = hcd_to_ehci(hcd);

		temp = ehci_readl(ehci, &ehci->regs->port_status[0]);

		if (temp & PORT_CONNECT) {
			temp = readl(SMU_CKRQMODE_MASK1);
			writel(temp & ~0x04000000, SMU_CKRQMODE_MASK1);
		} else {
			temp = readl(SMU_CKRQMODE_MASK1);
			writel(temp | 0x04000000, SMU_CKRQMODE_MASK1);
		}
	}

	return status;
}

/*-------------------------------------------------------------------------*/
/*
 * Root HUB Relinquish Port
*/
static void _emxx_relinquish_port(struct usb_hcd *hcd, int portnum)
{
	struct emxx_hcd_work *p_work = pg_hcd_work;

	if ((p_work->handed_over == 0) && (p_work->relinquish_count == 0)) {
		pr_debug(" --- Reset Host Controller\n");
		p_work->relinquish_count++;
		_emxx_vbus_control(0);
		disable_irq(INT_USB_OCI);
		emxx_hc_fatal_recovery(hcd);

		return;
	}

	p_work->handed_over = 0;
	p_work->relinquish_count = 0;
	ehci_relinquish_port(hcd, portnum);
}

/*-------------------------------------------------------------------------*/
/*
 * Root HUB Handed Over
*/
static int _emxx_port_handed_over(struct usb_hcd *hcd, int portnum)
{
	struct emxx_hcd_work *p_work = pg_hcd_work;

	p_work->handed_over = 1;

	return ehci_port_handed_over(hcd, portnum);
}

/*-------------------------------------------------------------------------*/
/*
 * USB Port Over Current Check
*/
static int _emxx_over_current_check(void)
{
	if (gpio_get_value(GPIO_USB_OCI) == 0) {
		_emxx_vbus_control(0);
		err("***** USB Over Current !!\n");
		return 1;
	} else
		return 0;
}

/*-------------------------------------------------------------------------*/
/*
 * Root HUB Control
*/
static int _emxx_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char	*buf,
	u16		wLength
)
{
	int		retval;

#ifdef CONFIG_USB_EHCI_TEST_MODE
	if (pg_hcd_work->hcd == NULL)
		pg_hcd_work->hcd = hcd;
#endif	/* CONFIG_USB_EHCI_TEST_MODE */

	retval = ehci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);

	/*---------------------------------------------------------------*/
	/* VBUS ON/OFF control */
	if (retval == 0) {
		if (wValue == USB_PORT_FEAT_POWER) {
			if (typeReq == SetPortFeature) {
				_emxx_vbus_control(1);
				mdelay(OVER_CURRENT_TIME);
				if (_emxx_over_current_check() == 0)
					enable_irq(INT_USB_OCI);
			} else if (typeReq == ClearPortFeature) {
				if (pg_hcd_work->vbus_flag) {
					disable_irq(INT_USB_OCI);
					_emxx_vbus_control(0);
				} else
					pg_hcd_work->vbus_flag = 1;
			}
		}
	}

	return retval;
}

/*-------------------------------------------------------------------------*/
/*
 * Over Current (GPIO118) Interrupt
*/
static irqreturn_t _emxx_over_current_irq(int irq, void *_hcd)
{
	_emxx_over_current_check();

	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/
/*
 * Save EHCI Register
*/
static void _emxx_ehci_save_register(struct usb_hcd *hcd)
{
	struct emxx_ehci_reg *p_reg;
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);

	p_reg = &pg_hcd_work->ehci_reg;
	if (p_reg->save_flag)
		return;

	p_reg->save_flag = 1;

	p_reg->USBINTR = ehci_readl(ehci, &ehci->regs->intr_enable);
	p_reg->USBCMD = ehci_readl(ehci, &ehci->regs->command);
	p_reg->USBSTS = ehci_readl(ehci, &ehci->regs->status);
	p_reg->FRINDEX = ehci_readl(ehci, &ehci->regs->frame_index);
	p_reg->PERIODICLISTBASE = ehci_readl(ehci, &ehci->regs->frame_list);
	p_reg->ASYNCLISTADDR = ehci_readl(ehci, &ehci->regs->async_next);
	p_reg->CONFIGFLAG = ehci_readl(ehci, &ehci->regs->configured_flag);
	p_reg->PORTSC = ehci_readl(ehci, &ehci->regs->port_status[0]);
}

/*-------------------------------------------------------------------------*/
/*
 * Restore EHCI Register
*/
static void _emxx_ehci_load_register(struct usb_hcd *hcd)
{
	struct emxx_ehci_reg *p_reg;
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);

	p_reg = &pg_hcd_work->ehci_reg;
	if (p_reg->save_flag == 0)
		return;

	ehci_writel(ehci, p_reg->CONFIGFLAG, &ehci->regs->configured_flag);
	ehci_writel(ehci, p_reg->PORTSC, &ehci->regs->port_status[0]);
	ehci_writel(ehci, p_reg->ASYNCLISTADDR, &ehci->regs->async_next);
	ehci_writel(ehci, p_reg->PERIODICLISTBASE, &ehci->regs->frame_list);
	ehci_writel(ehci, p_reg->FRINDEX, &ehci->regs->frame_index);
	ehci_writel(ehci, p_reg->USBSTS, &ehci->regs->status);
	ehci_writel(ehci, p_reg->USBCMD, &ehci->regs->command);
	ehci_writel(ehci, p_reg->USBINTR, &ehci->regs->intr_enable);

	p_reg->save_flag = 0;
}

static struct delayed_work emxx_ehci_wr;

static void _emxx_ehci_work(struct work_struct *work)
{
	u32 status, temp;

	status = emxx_io_read32(EMXX_EHCI_BASE + 0x64);

	temp = readl(SMU_CKRQMODE_MASK1);
	if (status & PORT_CONNECT)
		writel(temp & ~0x04000000, SMU_CKRQMODE_MASK1);
	else
		writel(temp | 0x04000000, SMU_CKRQMODE_MASK1);
}

/*-------------------------------------------------------------------------*/
/*
 * EHCI Probe
*/
static int _emxx_hcd_probe(
	const struct hc_driver *driver,
	struct platform_device *pdev
)
{
	int		retval;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;

	retval = _emxx_init(pdev);
	if (retval != 0) {
		dev_err(&pdev->dev, "***** _emxx_init() Error !!\n");
		return retval;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, "emxx_ehci");
	if (!hcd) {
		retval = -ENOMEM;
		goto err0;
	}

	hcd->rsrc_start = EHCI_BASE;
	hcd->rsrc_len = EHCI_SIZE;
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("request_mem_region failed");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		pr_debug("ioremap failed");
		retval = -ENOMEM;
		goto err2;
	}

	set_irq_type(INT_USB_OCI, IRQ_TYPE_EDGE_BOTH);
	retval = request_irq(INT_USB_OCI,
				_emxx_over_current_irq,
				IRQF_SHARED,
				"Over Current",
				hcd);
	if (retval != 0) {
		pr_debug("request_irq failed");
		goto err3;
	}
	disable_irq(INT_USB_OCI);

	_emxx_hc_start(hcd); /* after ioremap */

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);
	ehci->sbrn = 0x20;
	hcd->has_tt = 0;

	INIT_DELAYED_WORK(&emxx_ehci_wr, _emxx_ehci_work);

	/* ehci_hcd_init(hcd_to_ehci(hcd)); */
	retval = usb_add_hcd(hcd, INT_USBH, IRQF_DISABLED | IRQF_SHARED);
	if (retval == 0)
		return retval;

	_emxx_hc_stop(hcd);

	free_irq(INT_USB_OCI, hcd);

err3:
	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	usb_put_hcd(hcd);
err0:
	/* free Work area */
	_emxx_exit(pdev);

	return retval;
}

/*-------------------------------------------------------------------------*/
/*
 * EHCI Remve
*/
static void _emxx_hcd_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
	usb_remove_hcd(hcd);
	_emxx_hc_stop(hcd);
	free_irq(INT_USB_OCI, hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	/* free Work area */
	_emxx_exit(pdev);
}

static const struct hc_driver ehci_nbu2sshc_hc_driver = {
	.description = hcd_name,
	.product_desc = "EMXX EHCI",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_init,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
#ifdef CONFIG_USB_EHCI_TEST_MODE
	.urb_enqueue = _emxx_ehci_urb_enqueue,
#else	/* !CONFIG_USB_EHCI_TEST_MODE */
	.urb_enqueue = ehci_urb_enqueue,
#endif	/* CONFIG_USB_EHCI_TEST_MODE */
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = _emxx_hub_status_data,
	.hub_control = _emxx_hub_control,

#ifdef	CONFIG_PM
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
#endif

	.relinquish_port = _emxx_relinquish_port,
	.port_handed_over = _emxx_port_handed_over,
};

static int ehci_hcd_drv_probe(struct platform_device *pdev)
{
	if (usb_disabled())
		return -ENODEV;

	return _emxx_hcd_probe(&ehci_nbu2sshc_hc_driver, pdev);
}

static int ehci_hcd_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	_emxx_hcd_remove(hcd, pdev);

	return 0;
}

#ifdef CONFIG_PM
static int ehci_hcd_drv_suspend(
	struct platform_device *pdev,
	pm_message_t state
)
{
	u32		tmp;
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	cancel_delayed_work(&emxx_ehci_wr);

	tmp = emxx_io_read32(EMXX_CONFIGFLAG);
	if (tmp != EMXX_CONFIG_FLAG)
		return 0;

	disable_irq(INT_USB_OCI);
	_emxx_vbus_control(0);
	_emxx_ehci_save_register(hcd);
	_emxx_hc_stop(hcd);

	return 0;
}

static int ehci_hcd_drv_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	_emxx_hc_start(hcd);
	_emxx_ehci_load_register(hcd);
	_emxx_vbus_control(1);

	writel(readl(SMU_CKRQMODE_MASK1) & ~0x04000000, SMU_CKRQMODE_MASK1);
	schedule_delayed_work(&emxx_ehci_wr, msecs_to_jiffies(200));
	return 0;
}

void emxx_ehci_idle_suspend(void)
{
	u32 val;

	val = emxx_io_read32(EMXX_EHCI_BASE + 0x64);
	emxx_io_write32(EMXX_EHCI_BASE + 0x64, val | 0x700000);

	val = emxx_io_read32(EHCI_PM_CONTROL);
	emxx_io_write32(PM_CONTROL, val | (1 << 8));

	val = emxx_io_read32(PM_CONTROL) & ~0x3;
	emxx_io_write32(PM_CONTROL, val | (1 << 1));

	val = emxx_io_read32(USBCTR);
	emxx_io_write32(USBCTR, val | PCICLK_MASK);
}

void emxx_ehci_idle_resume(void)
{
	u32 val;

	val = emxx_io_read32(USBCTR);
	emxx_io_write32(USBCTR, val & ~PCICLK_MASK);

	val = emxx_io_read32(PM_CONTROL);
	emxx_io_write32(PM_CONTROL, val & ~0x3);

	val = emxx_io_read32(EHCI_PM_CONTROL) & ~(1 << 8);
	emxx_io_write32(EHCI_PM_CONTROL, val | (1 << 15));
}

#endif	/* CONFIG_PM */

static struct platform_driver ehci_hcd_emxx_driver = {
	.probe = ehci_hcd_drv_probe,
	.remove = ehci_hcd_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
#ifdef CONFIG_PM
	.suspend	= ehci_hcd_drv_suspend,
	.resume		= ehci_hcd_drv_resume,
#endif
	.driver = {
			.name = "emxx-ehci-driver",
			.bus = &platform_bus_type
	}
};

MODULE_ALIAS("platform:emxx-ehci-driver");
