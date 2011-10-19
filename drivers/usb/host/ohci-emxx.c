/*
 *  OHCI HCD (Host Controller Driver) for EMXX USB.
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

#include <mach/hardware.h>
#include <mach/pmu.h>
#include <mach/smu.h>
#include <mach/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include "ehci-emxx.h"


#define OHCI_BASE				EMXX_USBS0_BASE
#define OHCI_SIZE				0x1000

/* ----- Work ----- */
struct emxx_hcd_work	*pg_ohci_work;


#ifdef CONFIG_USB_OHCI_TEST_MODE
	#include "ohci-emxx_test.c"
#endif	/* CONFIG_USB_OHCI_TEST_MODE */


/*-------------------------------------------------------------------------*/
/*
 * initialize
 *   initialize for Work area
*/
static int _emxx_init(struct platform_device *pdev)
{
	int	nret = 0;
	struct emxx_hcd_work	*p_work;

	pg_ohci_work = NULL;	/* global */

	p_work = kzalloc(sizeof(struct emxx_hcd_work), GFP_KERNEL);
	if (p_work != NULL) {
		pg_ohci_work = p_work;

		spin_lock_init(&p_work->lock);
		p_work->dma_mask = DMA_32BIT_MASK;

		/*-----------------------------------------------------------*/
		/* DMA parameter setting */
		pdev->dev.dma_mask = &p_work->dma_mask;
		pdev->dev.coherent_dma_mask = DMA_32BIT_MASK;

#ifdef CONFIG_USB_OHCI_TEST_MODE
		nret = _emxx_test_mode_init(p_work);
#endif	/* CONFIG_USB_OHCI_TEST_MODE */

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

	if (pg_ohci_work != NULL) {

#ifdef CONFIG_USB_OHCI_TEST_MODE
		_emxx_test_mode_exit(pg_ohci_work);
#endif	/* CONFIG_USB_OHCI_TEST_MODE */

		kfree(pg_ohci_work);
		pg_ohci_work = NULL;
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
	struct emxx_hcd_work *p_work = pg_ohci_work;

	if (p_work == NULL)
		return -ENODEV;

	/* Clock & Reset & Direct Power Down */
	data = emxx_io_read32(USBCTR);
	data &= ~(DIRPD);
	emxx_io_write32(USBCTR, data);
#ifdef CONFIG_MACH_EMEV
	data &= ~(PCICLK_MASK | USBH_RST);
#elif defined(CONFIG_MACH_EMGR)
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

	/****** AHB-PCI Bridge Communication Registers ******/
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
	data = PCIWIN1_PCICMD | AHB_CFG_HOST;
	emxx_io_write32(AHBPCI_WIN1_CTR, data);

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

	gpio_direction_output(USB_VBUS_GPIO, 0);
}

static void _emxx_hc_start(struct usb_hcd *hcd)
{
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
	_ahb_pci_bridge_init();
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
		pg_ohci_work->vbus_flag = 1;
	}
}

/*-------------------------------------------------------------------------*/
/*
 * Root HUB Status Data
*/
static int _emxx_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	int		status;

	status = ohci_hub_status_data(hcd, buf);

	if (status) {
		u32		temp;
		struct ohci_hcd	*ohci = hcd_to_ohci(hcd);

		temp = ohci_readl(ohci, &ohci->regs->roothub.portstatus[0]);

		if (temp & RH_PS_CCS) {
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
	if (pg_ohci_work->hcd == NULL)
		pg_ohci_work->hcd = hcd;
#endif	/* CONFIG_USB_EHCI_TEST_MODE */

	retval = ohci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);

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
				if (pg_ohci_work->vbus_flag) {
					disable_irq(INT_USB_OCI);
					_emxx_vbus_control(0);
				} else
					pg_ohci_work->vbus_flag = 1;
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
 * Save OHCI Register
*/
static void _emxx_ohci_save_register(struct usb_hcd *hcd)
{
	struct emxx_ohci_reg *p_reg;
	struct ohci_hcd		*ohci = hcd_to_ohci(hcd);

	p_reg = &pg_ohci_work->ohci_reg;
	if (p_reg->save_flag)
		return;

	p_reg->save_flag = 1;

	p_reg->HcInterruptEnable = ohci_readl(ohci, &ohci->regs->intrenable);
	p_reg->HcControl = ohci_readl(ohci, &ohci->regs->control);
	p_reg->HcCommandStatus = ohci_readl(ohci, &ohci->regs->cmdstatus);
	p_reg->HcHCCA = ohci_readl(ohci, &ohci->regs->hcca);
	p_reg->HcControlHeadED =
		 ohci_readl(ohci, &ohci->regs->ed_controlhead);
	p_reg->HcControlCurrentED =
		 ohci_readl(ohci, &ohci->regs->ed_controlcurrent);
	p_reg->HcBulkHeadED = ohci_readl(ohci, &ohci->regs->ed_bulkhead);
	p_reg->HcBulkCurrentED =
		 ohci_readl(ohci, &ohci->regs->ed_bulkcurrent);
	p_reg->HcFmInterval = ohci_readl(ohci, &ohci->regs->fminterval);
	p_reg->HcPeriodicStart = ohci_readl(ohci, &ohci->regs->periodicstart);
	p_reg->HcLSThreshold = ohci_readl(ohci, &ohci->regs->lsthresh);

	p_reg->HcRhDescrptorA = ohci_readl(ohci, &ohci->regs->roothub.a);
	p_reg->HcRhDescrptorB = ohci_readl(ohci, &ohci->regs->roothub.b);
	p_reg->HcRhStatus = ohci_readl(ohci, &ohci->regs->roothub.status);
	p_reg->HcRhPortStatus =
		 ohci_readl(ohci, &ohci->regs->roothub.portstatus[0]);
}

/*-------------------------------------------------------------------------*/
/*
 * Restore OHCI Register
*/
static void _emxx_ohci_load_register(struct usb_hcd *hcd)
{
	struct emxx_ohci_reg *p_reg;
	struct ohci_hcd		*ohci = hcd_to_ohci(hcd);

	p_reg = &pg_ohci_work->ohci_reg;
	if (p_reg->save_flag == 0)
		return;

	ohci_writel(ohci, p_reg->HcRhPortStatus,
		 &ohci->regs->roothub.portstatus[0]);
	ohci_writel(ohci, p_reg->HcRhStatus, &ohci->regs->roothub.status);
	ohci_writel(ohci, p_reg->HcRhDescrptorB, &ohci->regs->roothub.b);
	ohci_writel(ohci, p_reg->HcRhDescrptorA, &ohci->regs->roothub.a);

	ohci_writel(ohci, p_reg->HcLSThreshold, &ohci->regs->lsthresh);
	ohci_writel(ohci, p_reg->HcPeriodicStart, &ohci->regs->periodicstart);
	ohci_writel(ohci, p_reg->HcFmInterval, &ohci->regs->fminterval);
	ohci_writel(ohci, p_reg->HcBulkCurrentED, &ohci->regs->ed_bulkcurrent);
	ohci_writel(ohci, p_reg->HcBulkHeadED, &ohci->regs->ed_bulkhead);
	ohci_writel(ohci, p_reg->HcControlCurrentED,
		 &ohci->regs->ed_controlcurrent);
	ohci_writel(ohci, p_reg->HcControlHeadED,
		 &ohci->regs->ed_controlhead);
	ohci_writel(ohci, p_reg->HcHCCA, &ohci->regs->hcca);
	ohci_writel(ohci, p_reg->HcCommandStatus, &ohci->regs->cmdstatus);
	ohci_writel(ohci, p_reg->HcControl, &ohci->regs->control);
	ohci_writel(ohci, p_reg->HcInterruptEnable, &ohci->regs->intrenable);

	p_reg->save_flag = 0;
}

/*-------------------------------------------------------------------------*/
/*
 * OHCI Probe
*/
static int _emxx_ohci_probe(const struct hc_driver *driver,
			 struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd;

	retval = _emxx_init(pdev);
	if (retval != 0) {
		dev_err(&pdev->dev, "***** _emxx_init() Error !!\n");
		return retval;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, "emxx_ohci");
	if (hcd == NULL) {
		retval = -ENOMEM;
		goto err0;
	}

	hcd->rsrc_start = OHCI_BASE;
	hcd->rsrc_len = OHCI_SIZE;
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		usb_put_hcd(hcd);
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

	_emxx_hc_start(hcd);

	ohci_hcd_init(hcd_to_ohci(hcd));

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
	_emxx_exit(pdev);

	return retval;
}

/*-------------------------------------------------------------------------*/
/*
 * OHCI Remve
*/
static void _emxx_ohci_remove(struct usb_hcd *hcd,
			struct platform_device *pdev)
{
	usb_remove_hcd(hcd);
	_emxx_hc_stop(hcd);
	free_irq(INT_USB_OCI, hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	_emxx_exit(pdev);
}

static int __devinit ohci_emxx_start(struct usb_hcd *hcd)
{
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);
	int		ret;

	ret = ohci_init(ohci);
	if (ret < 0)
		return ret;

	ret = ohci_run(ohci);
	if (ret < 0) {
		err("can't start %s", hcd->self.bus_name);
		ohci_stop(hcd);
		return ret;
	}

	return 0;
}

static struct hc_driver ohci_nbu2sshc_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "EMXX OHCI",
	.hcd_priv_size		= sizeof(struct ohci_hcd),
	.irq				= ohci_irq,
	.flags				= HCD_USB11 | HCD_MEMORY,
	.start				= ohci_emxx_start,
	.stop				= ohci_stop,
	.shutdown			= ohci_shutdown,

#ifdef CONFIG_USB_OHCI_TEST_MODE
	.urb_enqueue = _emxx_ohci_urb_enqueue,
#else	/* !CONFIG_USB_OHCI_TEST_MODE */
	.urb_enqueue		= ohci_urb_enqueue,
#endif	/* CONFIG_USB_OHCI_TEST_MODE */
	.urb_dequeue		= ohci_urb_dequeue,
	.endpoint_disable	= ohci_endpoint_disable,
	.get_frame_number	= ohci_get_frame,
	.hub_status_data	= _emxx_hub_status_data,
	.hub_control		= _emxx_hub_control,

#ifdef CONFIG_PM
	.bus_suspend		= ohci_bus_suspend,
	.bus_resume			= ohci_bus_resume,
#endif
	.start_port_reset	= ohci_start_port_reset,
};

static int ohci_hcd_drv_probe(struct platform_device *pdev)
{
	int ret;

	ret = -ENODEV;
	if (!usb_disabled())
		ret = _emxx_ohci_probe(&ohci_nbu2sshc_hc_driver, pdev);

	return ret;
}

static int ohci_hcd_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	_emxx_ohci_remove(hcd, pdev);

	return 0;
}

#ifdef CONFIG_PM
static int ohci_hcd_drv_suspend(
	struct platform_device *pdev,
	pm_message_t state
)
{
	u32		tmp;
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	tmp = emxx_io_read32(EMXX_CONFIGFLAG);
	if (tmp == EMXX_CONFIG_FLAG)
		return 0;

	disable_irq(INT_USB_OCI);
	_emxx_vbus_control(0);
	_emxx_ohci_save_register(hcd);
	_emxx_hc_stop(hcd);

	return 0;
}

static int ohci_hcd_drv_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	_emxx_hc_start(hcd);
	_emxx_ohci_load_register(hcd);
	_emxx_vbus_control(1);
	ohci_finish_controller_resume(hcd);

	return 0;
}

void emxx_ohci_idle_suspend(void)
{
	u32 val, usbctl;

	usbctl = emxx_io_read32(USBCTR);
	emxx_io_write32(USBCTR, usbctl & ~PCICLK_MASK);

#if 0
	val &= ~OHCI_CTRL_HCFS;
	emxx_io_write32(EMXX_OHCI_HCCONTROL, val | OHCI_USB_SUSPEND);

	val = emxx_io_read32(EMXX_OHCI_HCRHPORTSTATUS);
	emxx_io_write32(EMXX_OHCI_HCRHPORTSTATUS, val | RH_PS_PSS);
#endif

	val = emxx_io_read32(EMXX_OHCI_HCRHSTATUS);
	emxx_io_write32(EMXX_OHCI_HCRHSTATUS, val | RH_HS_DRWE);

	val = emxx_io_read32(PM_CONTROL);
	emxx_io_write32(PM_CONTROL, val | (1 << 8));

	if ((usbctl & PCICLK_MASK) == 0) {

		val = emxx_io_read32(PM_CONTROL) & ~0x3;
		emxx_io_write32(PM_CONTROL, val | (1 << 1));
	}
	emxx_io_write32(USBCTR, usbctl | PCICLK_MASK);
}

void emxx_ohci_idle_resume(void)
{
	u32 val, usbctl;

	usbctl = emxx_io_read32(USBCTR);
	if (usbctl & PCICLK_MASK) {
		emxx_io_write32(USBCTR, usbctl & ~PCICLK_MASK);

		val = emxx_io_read32(PM_CONTROL);
		emxx_io_write32(PM_CONTROL, val & ~0x3);
	}

	val = emxx_io_read32(PM_CONTROL) & ~(1 << 8);
	emxx_io_write32(PM_CONTROL, val | (1<<15));

#if 0
	val = emxx_io_read32(EMXX_OHCI_HCCONTROL) & ~OHCI_CTRL_HCFS;
	emxx_io_write32(EMXX_OHCI_HCCONTROL, val | OHCI_USB_OPER);
	val = emxx_io_read32(EMXX_OHCI_HCRHPORTSTATUS);
	emxx_io_write32(EMXX_OHCI_HCRHPORTSTATUS, val | RH_PS_PES);
#endif
}
#endif

static struct platform_driver ohci_hcd_emxx_driver = {
	.probe		= ohci_hcd_drv_probe,
	.remove		= ohci_hcd_drv_remove,
	.shutdown 	= usb_hcd_platform_shutdown,
#ifdef CONFIG_PM
	.suspend	= ohci_hcd_drv_suspend,
	.resume		= ohci_hcd_drv_resume,
#endif
	.driver		= {
		.name	= "emxx-ohci-driver",
	},
};

MODULE_ALIAS("platform:emxx-ohci-driver");
