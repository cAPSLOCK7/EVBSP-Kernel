/*
 *  ehci-emxx.h -- for emxx High speed USB Host controller.
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


#ifndef __EHCI_EMXX_H
#define __EHCI_EMXX_H


#define OHCI_BASE			EMXX_USBS0_BASE
#define OHCI_SIZE			0x1000

#define EHCI_BASE			(OHCI_BASE + OHCI_SIZE)
#define EHCI_SIZE			0x1000


/*-------------------------------------------------------*/
/* VBUS and OverCurrent */
#define USB_VBUS_GPIO		GPIO_USB_PPON
#define USB_OVER_CURRENT	GPIO_USB_OCI

#define OVER_CURRENT_TIME	5

/*-------------------------------------------------------*/
/* BIT */
#define BIT00		0x00000001
#define BIT01		0x00000002
#define BIT02		0x00000004
#define BIT03		0x00000008
#define BIT04		0x00000010
#define BIT05		0x00000020
#define BIT06		0x00000040
#define BIT07		0x00000080
#define BIT08		0x00000100
#define BIT09		0x00000200
#define BIT10		0x00000400
#define BIT11		0x00000800
#define BIT12		0x00001000
#define BIT13		0x00002000
#define BIT14		0x00004000
#define BIT15		0x00008000
#define BIT16		0x00010000
#define BIT17		0x00020000
#define BIT18		0x00040000
#define BIT19		0x00080000
#define BIT20		0x00100000
#define BIT21		0x00200000
#define BIT22		0x00400000
#define BIT23		0x00800000
#define BIT24		0x01000000
#define BIT25		0x02000000
#define BIT26		0x04000000
#define BIT27		0x08000000
#define BIT28		0x10000000
#define BIT29		0x20000000
#define BIT30		0x40000000
#define BIT31		0x80000000


#define BIT31_BIT11	0xFFFFF800
#define BIT31_BIT16	0xFFFF0000
#define BIT31_BIT28	0xF0000000
#define BIT23_BIT16	0x00FF0000
#define BIT15_BIT00	0x0000FFFF
#define BIT15_BIT04	0x0000FFF0
#define BIT11_BIT09	0x00000E00


#define BIT08_BIT06	0x000001C0
#define BIT03_BIT01	0x0000000E

#define EMXX_OHCI_BASE			(EMXX_USBS0_BASE)

#define EMXX_OHCI_HCREVISION		(EMXX_USBS0_BASE+0x0000)
#define EMXX_OHCI_HCCONTROL		(EMXX_USBS0_BASE+0x0004)
#define EMXX_OHCI_HCCOMMANDSTATUS	(EMXX_USBS0_BASE+0x0008)
#define EMXX_OHCI_HCINTERRUPTSTATUS	(EMXX_USBS0_BASE+0x000C)
#define EMXX_OHCI_HCINTERRUPTENABLE	(EMXX_USBS0_BASE+0x0010)
#define EMXX_OHCI_HCINTERRUPTDISABLE	(EMXX_USBS0_BASE+0x0014)
#define EMXX_OHCI_HCHCCA		(EMXX_USBS0_BASE+0x0018)
#define EMXX_OHCI_HCPERIODICCURRENTED	(EMXX_USBS0_BASE+0x001C)
#define EMXX_OHCI_HCCONTROLHEADED	(EMXX_USBS0_BASE+0x0020)
#define EMXX_OHCI_HCCONTROLCURRENTED	(EMXX_USBS0_BASE+0x0024)
#define EMXX_OHCI_HCBULKHEADED		(EMXX_USBS0_BASE+0x0028)
#define EMXX_OHCI_HCBULKCURRENTED	(EMXX_USBS0_BASE+0x002C)
#define EMXX_OHCI_HCDONEHEAD		(EMXX_USBS0_BASE+0x0030)
#define EMXX_OHCI_HCFMINTERVAL		(EMXX_USBS0_BASE+0x0034)
#define EMXX_OHCI_HCFMREMAINING		(EMXX_USBS0_BASE+0x0038)
#define EMXX_OHCI_HCFMNUMBER		(EMXX_USBS0_BASE+0x003C)
#define EMXX_OHCI_HCPERIODICSTART	(EMXX_USBS0_BASE+0x0040)
#define EMXX_OHCI_HCLSTHRESHOLD		(EMXX_USBS0_BASE+0x0044)
#define EMXX_OHCI_HCRHDESCRPTORA	(EMXX_USBS0_BASE+0x0048)
#define EMXX_OHCI_HCRHDESCRPTORB	(EMXX_USBS0_BASE+0x004C)
#define EMXX_OHCI_HCRHSTATUS		(EMXX_USBS0_BASE+0x0050)
#define EMXX_OHCI_HCRHPORTSTATUS	(EMXX_USBS0_BASE+0x0054)


#define EMXX_EHCI_BASE			(EMXX_USBS0_BASE+0x1000)
#define EMXX_CONFIGFLAG			(EMXX_EHCI_BASE + 0x0060)


/* PCI Configuration Registers for OHCI/EHCI */
#define PCI_CONF_OHCI_BASE		(EMXX_USBS0_BASE+0x10000)
#define OHCI_VID_DID			(PCI_CONF_OHCI_BASE + 0x0000)
#define OHCI_CMND_STS			(PCI_CONF_OHCI_BASE + 0x0004)
#define OHCI_BASEAD			(PCI_CONF_OHCI_BASE + 0x0010)

#define PCI_CONF_EHCI_BASE		(EMXX_USBS0_BASE+0x10100)
#define EHCI_VID_DID			(PCI_CONF_EHCI_BASE + 0x0000)
#define EHCI_CMND_STS			(PCI_CONF_EHCI_BASE + 0x0004)
#define EHCI_BASEAD			(PCI_CONF_EHCI_BASE + 0x0010)
#define EHCI_PM_CONTROL			(PCI_CONF_EHCI_BASE + 0x0044)

/* PCI Configuration Registers for AHB-PCI Bridge Registers */
#define PCI_CONF_AHBPCI_BAS		(EMXX_USBS0_BASE+0x10000)
#define VID_DID				(PCI_CONF_AHBPCI_BAS + 0x0000)
#define CMND_STS			(PCI_CONF_AHBPCI_BAS + 0x0004)
#define REVID_CC			(PCI_CONF_AHBPCI_BAS + 0x0008)
#define CLS_LT_HT_BIST			(PCI_CONF_AHBPCI_BAS + 0x000C)
#define BASEAD				(PCI_CONF_AHBPCI_BAS + 0x0010)
#define WIN1_BASEAD			(PCI_CONF_AHBPCI_BAS + 0x0014)
#define WIN2_BASEAD			(PCI_CONF_AHBPCI_BAS + 0x0018)

#define SSVID_SSID			(PCI_CONF_AHBPCI_BAS + 0x002C)

#define INTR_LINE_PIN			(PCI_CONF_AHBPCI_BAS + 0x003C)

#define PM_CONTROL			(PCI_CONF_AHBPCI_BAS + 0x0044)

#define TRANSCIVER_CHARACTERISTIC	(PCI_CONF_AHBPCI_BAS + 0x00F0)

/* AHB-PCI Bridge PCI Communication Registers */
#define AHBPCI_BASE			(EMXX_USBS0_BASE+0x10800)
#define PCIAHB_WIN1_CTR			(AHBPCI_BASE + 0x0000)
#define PCIAHB_WIN2_CTR			(AHBPCI_BASE + 0x0004)
#define PCIAHB_DCT_CTR			(AHBPCI_BASE + 0x0008)

#define AHBPCI_WIN1_CTR			(AHBPCI_BASE + 0x0010)
#define AHBPCI_WIN2_CTR			(AHBPCI_BASE + 0x0014)

#define AHBPCI_DCT_CTR			(AHBPCI_BASE + 0x001C)
#define PCI_INT_ENABLE			(AHBPCI_BASE + 0x0020)
#define PCI_INT_STATUS			(AHBPCI_BASE + 0x0024)

#define AHB_BUS_CTR			(AHBPCI_BASE + 0x0030)
#define USBCTR				(AHBPCI_BASE + 0x0034)

#define PCI_ARBITER_CTR			(AHBPCI_BASE + 0x0040)

#define PCI_UNIT_REV			(AHBPCI_BASE + 0x004C)


/* ------ EMXX_CONFIGFLAG (0x1060) */
#define EMXX_CONFIG_FLAG				BIT00

/* ------ VID_DID (0x10000) */
#define DEVICE_ID			BIT31_BIT16		/* R */
#define VENDOR_ID			BIT15_BIT00		/* R */

/* ------ CMND_STS (0x10004) */
#define DETPERR				BIT31			/* RW */
#define SIGSERR				BIT30			/* RW */
#define REMABORT			BIT29			/* RW */
#define RETABORT			BIT28			/* RW */
#define SIGTABORT			BIT27			/* RW */
#define DEVTIM				(BIT26|BIT25)		/* R */
#define MDPERR				BIT24			/* R */
#define FBTBCAP				BIT23			/* R */

#define _66MCAP				BIT21			/* R */
#define CAPLIST				BIT20			/* R */

#define FBTBEN				BIT09			/* R */
#define SERREN				BIT08			/* RW */
#define STEPCTR				BIT07			/* R */
#define PERREN				BIT06			/* RW */
#define VGAPSNP				BIT05			/* R */
#define MWINVEN				BIT04			/* R */
#define SPECIALC			BIT03			/* R */
#define MASTEREN			BIT02			/* RW */
#define MEMEN				BIT01			/* RW */
#define IOEN				BIT00			/* R */

/* ------ WIN1_BASEAD (0x10014) */
#define PCI_WIN1_BASEADR		(BIT31|BIT30)		/* RW */

#define WIN1_PREFETCH			BIT03			/* R */
#define WIN1_TYPE			(BIT02|BIT01)		/* R */
#define WIN1_MEM			BIT00			/* R */

/* ------ WIN2_BASEAD (0x10018) */
#define PCI_WIN2_BASEADR		BIT31_BIT28		/* RW */

#define WIN2_PREFETCH			BIT03			/* R */
#define WIN2_TYPE			(BIT02|BIT01)		/* R */
#define WIN2_MEM			BIT00			/* R */


/* ------ TRANSCIVER_CHARACTERISTIC (0x100F0) */
#define PORT1_SQU			0x000000F0		/* RW */
#define PORT1_HSIUP			0x0000000F		/* RW */

#define USB_PORT1_SQU			(BIT06+BIT05+BIT04)
#define USB_PORT1_HSIUP			BIT03

/* ------ PCIAHB_WIN1_CTR (0x10800) */
#define AHB_BASEADR			(BIT31|BIT30)		/* RW */

#define ENDIAN_CTR			BIT08_BIT06		/* RW */

#define PREFETCH			(BIT01|BIT00)		/* RW */

/* ------ PCIAHB_WIN2_CTR (0x10804) */
#define AHB_BASE_ADR			BIT31_BIT28		/* RW */

#define ENDIAN_CTR			BIT08_BIT06		/* RW */

#define PREFETCH			(BIT01|BIT00)		/* RW */

/* ------ PCIAHB_DCT_CTR (0x10808) */
#define PCIAHB_DISCARD_TIMER		BIT15_BIT04		/* RW */

#define DISCARD_EN			BIT00			/* RW */

/* ------ AHBPCI_WIN1_CTR (0x10810) */
#define PCIWIN1_BASEADR			BIT31_BIT11		/* RW */

#define ENDIAN_CTR			BIT08_BIT06		/* RW */

#define CFGTYPE				BIT04			/* RW */
#define PCICMD				BIT03_BIT01		/* RW */
	#define PCIWIN1_PCICMD		(BIT03|BIT01)

#define AHB_CFG_AHBPCI			0x40000000
#define AHB_CFG_HOST			0x80000000

/* ------ AHBPCI_WIN2_CTR (0x10814) */
#define PCIWIN2_BASEADR			BIT31_BIT16		/* RW */

#define ENDIAN_CTR			BIT08_BIT06		/* RW */
#define BURST_EN			BIT05			/* RW */
#define PCICMD				BIT03_BIT01		/* RW */
	#define PCIWIN2_PCICMD		(BIT02|BIT01)
#define PCIWIN2_PREFETCH		BIT00			/* RW */

/* ------ AHBPCI_DCT_CTR (0x1081C) */
#define AHBPCI_DISCARD_TIMER		BIT15_BIT04		/* RW */

#define DISCARD_EN			BIT00			/* RW */

/* ------ PCI_INT_ENABLE (0x10820) */
#define USBH_PMEEN			BIT19			/* RW */

#define USBH_INTBEN			BIT17			/* RW */
#define USBH_INTAEN			BIT16			/* RW */

#define AHBPCI_WIN_INTEN		BIT14			/* RW */
#define PCIAHB_WIN2_INTEN		BIT13			/* RW */
#define PCIAHB_WIN1_INTEN		BIT12			/* RW */

#define DMA_AHBPCI_INTEN		BIT09			/* RW */
#define DMA_PCIAHB_INTEN		BIT08			/* RW */

#define RESERR_INTEN			BIT05			/* RW */
#define SIGSERR_INTEN			BIT04			/* RW */
#define PERR_INTEN			BIT03			/* RW */
#define REMABORT_INTEN			BIT02			/* RW */
#define RETABORT_INTEN			BIT01			/* RW */
#define SIGTABORT_INTEN			BIT00			/* RW */

/* ------ PCI_INT_STATUS (0x10824) */
#define USBH_PME			BIT19			/* R */

#define USBH_INTB			BIT17			/* R */
#define USBH_INTA			BIT16			/* R */

#define AHBPCI_WIN_INT			BIT14			/* RW */
#define PCIAHB_WIN2_INT			BIT13			/* RW */
#define PCIAHB_WIN1_INT			BIT12			/* RW */

#define DMA_AHBPCI_INT			BIT09			/* RW */
#define DMA_PCIAHB_INT			BIT08			/* RW */

#define RESERR_INT			BIT05			/* RW */
#define SIGSERR_INT			BIT04			/* RW */
#define PERR_INT			BIT03			/* RW */
#define REMABORT_INT			BIT02			/* RW */
#define RETABORT_INT			BIT01			/* RW */
#define SIGTABORT_INT			BIT00			/* RW */

/* ------ AHB_BUS_CTR (0x10830) */
#define SMODE_READY_CTR			BIT17			/* RW */
#define SMODE_READ_BURST		BIT16			/* RW */

#define MMODE_HBUSREQ			BIT07			/* RW */
#define MMODE_BOUNDARY			(BIT06|BIT05)		/* RW */
#define MMODE_BURST_WIDTH		(BIT04|BIT03)		/* RW */
#define MMODE_SINGLE_MODE		(BIT04|BIT03)		/* RW */
#define MMODE_WR_INCR			BIT02			/* RW */
#define MMODE_BYTE_BURST		BIT01			/* RW */
#define MMODE_HTRANS			BIT00			/* RW */

#define AHB_BUS_CTR_SET	\
		(BIT17 | BIT07 | BIT04 | BIT03 | BIT02 | BIT01)

/* ------ USBCTR (0x10834) */
#define TEMP0				BIT11_BIT09		/* RW */

#define DIRPD				BIT08			/* RW */

#ifdef CONFIG_MACH_EMEV
#define TEMP1				(BIT03|BIT02)		/* RW */
#define TEMP1_1				BIT03			/* RW */
#define TEMP1_0				BIT02			/* RW */
#elif defined(CONFIG_MACH_EMGR)
#define PLL_RST				BIT02			/* RW */
#endif
#define PCICLK_MASK			BIT01			/* RW */
#define USBH_RST			BIT00			/* RW */

/* ------ PCI_ARBITER_CTR (0x10840) */
#define PCIBUS_PARK_TIMER		BIT23_BIT16		/* RW */
	#define PCIBUS_PARK_TIMER_SET	0x00070000

#define PCIBP_MODE			BIT12			/* RW */

#define PCIREQ7				BIT07			/* RW */
#define PCIREQ6				BIT06			/* RW */
#define PCIREQ5				BIT05			/* RW */
#define PCIREQ4				BIT04			/* RW */
#define PCIREQ3				BIT03			/* RW */
#define PCIREQ2				BIT02			/* RW */
#define PCIREQ1				BIT01			/* RW */
#define PCIREQ0				BIT00			/* RW */

/* ------ PCI_UNIT_REV (0x10848) */
#define MAJOR_REVISION_ID		BIT31_BIT16		/* R */
#define MINOR_REVISION_ID		BIT15_BIT00		/* R */


/*-------------------------------------------------------------------------*/

struct emxx_ohci_reg {
	u32	HcRevision;		/* HcRevision */
	u32	HcControl;		/* HcControl */
	u32	HcCommandStatus;	/* HcCommandStatus */
	u32	HcInterruptStatus;	/* HcInterruptStatus */
	u32	HcInterruptEnable;	/* HcInterruptEnable */
	u32	HcInterruptDisable;	/* HcInterruptDisable */
	u32	HcHCCA;			/* HcHCCA */
	u32	HcPeriodicCurrentED;	/* HcPeriodicCurrentED */
	u32	HcControlHeadED;	/* HcControlHeadED */
	u32	HcControlCurrentED;	/* HcControlCurrentED */
	u32	HcBulkHeadED;		/* HcBulkHeadED */
	u32	HcBulkCurrentED;	/* HcBulkCurrentED */
	u32	HcDoneHead;		/* HcDoneHead */
	u32	HcFmInterval;		/* HcFmInterval */
	u32	HcFmRemaining;		/* HcFmRemaining */
	u32	HcFmNumber;		/* HcFmNumber */
	u32	HcPeriodicStart;	/* HcPeriodicStart */
	u32	HcLSThreshold;		/* HcLSThreshold */
	u32	HcRhDescrptorA;		/* HcRhDescrptorA */
	u32	HcRhDescrptorB;		/* HcRhDescrptorB */
	u32	HcRhStatus;		/* HcRhStatus */
	u32	HcRhPortStatus;		/* HcRhPortStatus */

	u32	save_flag;		/* Save Flag */
};

struct emxx_ehci_reg {
	u32	HCIVERSION;		/* HCIVERSION/CAPLENGTH */
	u32	HCSPARAMS;		/* HCSPARAMS */
	u32	HCCPARAMS;		/* HCCPARAMS */
	u32	HCSP_PORTROUTE;		/* HCSP_PORTROUTE */
	u32	USBCMD;			/* USBCMD */
	u32	USBSTS;			/* USBSTS */
	u32	USBINTR;		/* USBINTR */
	u32	FRINDEX;		/* FRINDEX */
	u32	CTRLDSSEGMENT;		/* CTRLDSSEGMENT */
	u32	PERIODICLISTBASE;	/* PERIODICLISTBASE */
	u32	ASYNCLISTADDR;		/* ASYNCLISTADDR */
	u32	Reserved_1[9];
	u32	CONFIGFLAG;		/* CONFIGFLAG */
	u32	PORTSC;			/* PORTSC */

	u32	save_flag;		/* Save Flag */
};


struct emxx_hcd_work {
	spinlock_t lock;

	u64		dma_mask;

	struct emxx_ohci_reg	ohci_reg;
	struct emxx_ehci_reg	ehci_reg;

	u32		vbus_flag;

	struct workqueue_struct *emxx_restart_workqueue;
	struct delayed_work emxx_restart_work;
	struct usb_hcd	*p_hcd;

	u32		relinquish_count;
	u32		handed_over;

#ifdef CONFIG_USB_EHCI_TEST_MODE
	struct usb_hcd	*hcd;

	u32			b_port_flag;		/* Port Flag */
	u32			open_count;		/* Open Counter */
	u32			test_mode_flag;		/* Test Mode Flag */
	u32			device_flag;
	u32			cmd;

	struct usb_device usb_device;
	struct urb	urb;

	struct usb_ctrlrequest dr;			/* USB Request Buffer */
	unsigned char	data[64];			/* USB Data Buffer */
	u32		transfer_count;
	gfp_t	mem_flags;
	struct ehci_qtd	*qtd;

#endif	/* CONFIG_USB_EHCI_TEST_MODE */
};


static inline u32 emxx_io_read32(u32 address)
{
	return __raw_readl(IO_ADDRESS(address));
}

static inline void emxx_io_write32(u32 address, u32 data)
{
	__raw_writel(data, IO_ADDRESS(address));
}


#ifdef CONFIG_USB_EHCI_TEST_MODE
/*-------------------------------------------------------------------------*/
static inline char *usb_debug_get_file_name(char *p_filename)
{
	int		pos, len, i;
	char *p;

	pos = 0;
	p = p_filename;
	len = strlen(p);

	for (i = 0; i < len; i++) {
		if (p[i] == '/')
			pos = i+1;
	}

	p = p + pos;

	return p;
}


#ifdef USE_DEBUG_MESSAGE_ALL

	#ifdef USE_DEBUG_MESSAGE
		#define DBG_MSG(format, arg...) \
			printk(KERN_DEBUG "%s : " format "\n", \
				usb_debug_get_file_name(__FILE__), ## arg)
	#else
		#define DBG_MSG(format, arg...) do {} while (0)
	#endif

	#define DBG_MSGG(format, arg...) do {} while (0)

	#ifdef USE_INFOMATION_MESSAGE
		#define INFO_MSG(format, arg...) \
			printk(KERN_INFO "%s : " format "\n", \
				usb_debug_get_file_name(__FILE__), ## arg)
	#else
		#define INFO_MSG(format, arg...) do {} while (0)
	#endif
	#define INFO_MSGG(format, arg...) do {} while (0)

#else	/* !USE_DEBUG_MESSAGE_ALL */

	#define DBG_MSG(format, arg...) do {} while (0)
	#define DBG_MSGG(format, arg...) do {} while (0)
	#define INFO_MSG(format, arg...) do {} while (0)
	#define INFO_MSGG(format, arg...) do {} while (0)

#endif	/* USE_DEBUG_MESSAGE_ALL */

#define ERR_MSG(format, arg...) \
	printk(KERN_ERR "%s : *** Error *** " format "\n", \
		usb_debug_get_file_name(__FILE__), ## arg)
#define WARN_MSG(format, arg...) \
	printk(KERN_WARNING "%s : === Warning === " format "\n", \
		usb_debug_get_file_name(__FILE__), ## arg)
#endif	/* CONFIG_USB_EHCI_TEST_MODE */

#endif /* __EHCI_EMXX_H */
