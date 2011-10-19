/*
 * File Name       : drivers/nts/emxx_ntshw.c
 * Function        : NTSC Driver (H/W Control)
 * Release Version : Ver 1.00
 * Release Date    : 2010.09.27
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


/********************************************************
 *  Include Files                                       *
 *******************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/irqs.h>
#include <mach/smu.h>
#include <mach/pmu.h>
#include <mach/emxx_mem.h>

#include <mach/gpio.h>

#include "../video/emxx/emxx_common.h"
#include "ntsc.h"
#include "emxx_nts_common.h"
#include "emxx_nts_image.h"
#include "emxx_ntshw.h"


/********************************************************
 *  Definitions                                         *
 *******************************************************/
#define DEV_NAME "emxx_ntshw"


/********************************************************
 *  debug parameters                                    *
 *******************************************************/
#define _DEBUG_NTSHW  0x00 /* 00008421(bit) */
			   /* 0x01: debug function in
			    * 0x02: debug function out
			    * 0x08: debug frame change
			    * 0x40: debug FBIOBLANK
			    */


#define printk_err(fmt, arg...) \
	do {                     \
		printk(KERN_ERR DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#define printk_wrn(fmt, arg...) \
	do {                     \
		printk(KERN_WARNING DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#define printk_info(fmt, arg...) \
	do {                      \
		printk(KERN_INFO DEV_NAME ": %s: " fmt, ## arg); \
	} while (0)

#if _DEBUG_NTSHW
#define printk_dbg(level, fmt, arg...) \
	do {                            \
		if (level > 0) \
			printk(KERN_DEBUG DEV_NAME ": %s: " fmt, \
				__func__, ## arg); \
	} while (0)
#else
#define printk_dbg(level, fmt, arg...) \
	;
#endif


/********************************************************
 *  Macros                                              *
 *******************************************************/


/********************************************************
 * NTSC register initialize                             *
 *******************************************************/
#ifdef CONFIG_MACH_EMEV
#define CHG_PINSEL_NTSC_BIT  0x00000003
#define CHG_PINSEL_NTSC_INIT 0x00000000
#endif
#ifdef CONFIG_MACH_EMGR
#define CHG_PINSEL_AB_BIT  0x00000FFF
#define CHG_PINSEL_AB_INIT 0x00000001
#endif


/********************************************************
 *  Variables                                           *
 *******************************************************/


/********************************************************
 *  Structure                                           *
 *******************************************************/
struct emxx_ntshw {
	/* output mode */
	int           Outmode;

	/* NTSC MMIO */
	unsigned long NTSCMmio;
	char         *NTSCMmioV;

	/* NTSC framebuffer */
	unsigned long NtscFrameA_Y;
	unsigned long NtscFrameA_UV;
	unsigned long NtscFrameB_Y;
	unsigned long NtscFrameB_UV;
	unsigned long NtscFrameC_Y;
	unsigned long NtscFrameC_UV;
	unsigned long NtscFrameLength;
	char         *NtscFrameAV;
	char         *NtscFrameBV;

	/* Smem (common memory to NTS) */
	unsigned long Smem;
	unsigned long SmemLength;
	char         *SmemV;
};
static struct emxx_ntshw *ntshw;


/********************************************************
 *  Prototype declarations of local function            *
 *******************************************************/
/* ------------------ NTSC rgister set function ---------------------------- */
#ifdef NOT_USE_FUNC
static int           ntshw_set_control_upscale(int iSetUpscaleFlg);
#endif /* NOT_USE_FUNC */
static int           ntshw_set_control_outmode(int iSetOutmodeFlg);
static int           ntshw_set_areaad(int iSetAreaFlg, unsigned long ulSetAddrY,
 unsigned long ulSetAddrUV);
static int           ntshw_set_hoffset(unsigned long ulSetOffset);
static int           ntshw_set_register(int iSetOutmode);
/* ------------------ NTSC rgister check function -------------------------- */
#ifdef NOT_USE_FUNC
static unsigned long ntshw_chk_intrawstatus(void);
#endif /* NOT_USE_FUNC */
/* ------------------ NTSC initialize function ------------------------------ */
static int           ntshw_init_framebuf(struct emxx_nts_dev *ntsc);
static void          ntshw_exit_framebuf(void);
static void         *ntshw_init_val(void);





/* ------------------ NTSC H/W control function ----------------------------- */
/******************************************************************************
* MODULE   : ntshw_start
* FUNCTION : start NTS(H/W)
* RETURN   : none
* NOTE     : none
******************************************************************************/
void ntshw_start(int iStartFlg)
{
	if (iStartFlg == NTSHW_START) {
		printk_dbg((_DEBUG_NTSHW & 0x01), "NTSHW_START\n");
		ntshw_set_ntsout(NTS_NTSOUT_ENABLE_BLACK);
	} else {
		printk_dbg((_DEBUG_NTSHW & 0x01), "NTSHW_STOP\n");
		ntshw_set_ntsout(NTS_NTSOUT_DISABLE);
		while ((ntshw_chk_status() & NTS_STATUS_BIT)
		 != NTS_STATUS_DISABLE) {
			;
		}
	}
}


/******************************************************************************
* MODULE   : ntshw_reset
* FUNCTION : NTS(H/W) reset
* RETURN   : none
* NOTE     : none
******************************************************************************/
void ntshw_reset(int iResetFlg)
{
#ifdef CONFIG_MACH_EMGR
	u32	val;
#endif

	if (iResetFlg == NTSHW_RESET) {
		/* Reset NTSC */
		printk_dbg((_DEBUG_NTSHW & 0x41), "NTSHW_RESET\n");

		/* clock auto control -> OFF    */
		emxx_clkctrl_off(EMXX_CLKCTRL_NTS);
		emxx_clkctrl_off(EMXX_CLKCTRL_NTSPCLK);
		/* NTSC reset                   */
		emxx_reset_device(EMXX_RST_NTS);
		/* clock demand                 */
		emxx_close_clockgate(EMXX_CLK_NTS | EMXX_CLK_NTS_P);
#ifdef CONFIG_MACH_EMGR
		/* Disable NTSSCLK */
		writel(0, SMU_NTSSCLKDIV);
		writel(readl(SMU_PLL1CTRL1) | 0x2, SMU_PLL1CTRL1);
		writel(readl(SMU_OSC1CTRL1) | 0x2, SMU_OSC1CTRL1);
#endif
	} else {
#ifdef CONFIG_MACH_EMGR
		/* Enable NTSSCLK */
		val = readl(SMU_OSC1CTRL1);
		writel(val & ~0x2, SMU_OSC1CTRL1);
		if (val == 0xff) {
			while ((readl(SMU_PLL_STATUS) & 0x00100000) == 0)
				udelay(10);
		}
		writel(0x100104b3, SMU_PLL1CTRL0);
		val = readl(SMU_PLL1CTRL1);
		writel(val & ~0x2, SMU_PLL1CTRL1);
		if (val == 0xff) {
			while ((readl(SMU_PLL_STATUS) & 0x00000001) == 0)
				udelay(10);
		}
		writel(0x00000102, SMU_NTSSCLKDIV);
#endif
		/* UnReset NTSC */
		printk_dbg((_DEBUG_NTSHW & 0x41), "NTSHW_UNRESET\n");

		/* clock supply                 */
		emxx_open_clockgate(EMXX_CLK_NTS | EMXX_CLK_NTS_P);
		/* clock auto control -> OFF    */
		emxx_clkctrl_off(EMXX_CLKCTRL_NTS);
		emxx_clkctrl_off(EMXX_CLKCTRL_NTSPCLK);
		/* NTSC unreset                 */
		emxx_unreset_device(EMXX_RST_NTS);
		/* clock auto control -> ON     */
		emxx_clkctrl_on(EMXX_CLKCTRL_NTS);
		emxx_clkctrl_on(EMXX_CLKCTRL_NTSPCLK);
	}
}


/******************************************************************************
* MODULE   : ntshw_module_reset
* FUNCTION : NTS(H/W) reset
* RETURN   : none
* NOTE     : none
******************************************************************************/
void ntshw_module_reset(int iResetFlg)
{
	if (iResetFlg == NTSHW_RESET) {
		/* Reset NTSC Module */
		printk_dbg((_DEBUG_NTSHW & 0x01), "NTSHW_RESET\n");
		gpio_direction_output(GPIO_NTSC_RST, 0);
	} else {
		/* UnReset NTSC Module */
		printk_dbg((_DEBUG_NTSHW & 0x01), "NTSHW_UNRESET\n");
		gpio_direction_output(GPIO_NTSC_RST, 1);
	}
}


/* ------------------ NTSC rgister set function ----------------------------- */
/*****************************************************************************
* MODULE   : ntshw_set_interruput
* FUNCTION : enable NTSC VSYNC interrupt
* RETURN   : none
* NOTE     : none
******************************************************************************/
void ntshw_set_interruput(int iIntFlg)
{
	if (iIntFlg == NTSHW_INTENSET) {
		printk_dbg((_DEBUG_NTSHW & 0x01), "NTSHW_INTENSET\n");

		/* Interrupt Enable Set */
		writel(NTS_NTSVS_BIT, ntshw->NTSCMmioV + NTS_INTENSET);
	} else if (iIntFlg == NTSHW_INTENCLR) {
		printk_dbg((_DEBUG_NTSHW & 0x01), "NTSHW_INTENCLR\n");

		/* Interrupt Enable Clear */
		writel(NTS_INT_ALL_BIT, ntshw->NTSCMmioV + NTS_INTENCLR);
	} else {
		printk_dbg((_DEBUG_NTSHW & 0x01), "NTSHW_INTFFCLR\n");

		/* Interrupt Status Clear */
		writel(NTS_INT_ALL_BIT, ntshw->NTSCMmioV + NTS_INTFFCLR);
	}
}


/******************************************************************************
* MODULE   : ntshw_set_framesel
* FUNCTION : change NTSC display buffer
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
int ntshw_set_framesel(int iSetFrameNo)
{
	int iRet;
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");

	switch (iSetFrameNo) {
	case NTS_AREASEL_BUFA:	/* frame change -> A       FALL THROUGH */
	case NTS_AREASEL_BUFB:	/* frame change -> B       FALL THROUGH */
	case NTS_AREASEL_BUFC:	/* frame change -> C */
		writel(iSetFrameNo, ntshw->NTSCMmioV + NTS_FRAMESEL);
		iRet = 0;
		break;
	case NTS_AREASEL_BUFDISABLE:	/* FALL THROUGH */
	default:
		iRet = -EINVAL;
		break;
	}
	return iRet;
}


/******************************************************************************
* MODULE   : ntshw_set_ntsout
* FUNCTION : change NTSC output mode
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
int ntshw_set_ntsout(int iSetNtsoutFlg)
{
	int iRet;
	printk_dbg((_DEBUG_NTSHW & 0x01), "(%d)\n", iSetNtsoutFlg);

	switch (iSetNtsoutFlg) {
	case NTS_NTSOUT_ENABLE:
		/* ntsout -> external screen */
		iRet = 0;
		break;
	case NTS_NTSOUT_ENABLE_BLUE:
		/* ntsout -> blue screen     */
		printk_dbg((_DEBUG_NTSHW & 0x40),
		 "<set NTSOUT On (BlueBack)>\n");
		iRet = 0;
		break;
	case NTS_NTSOUT_ENABLE_BLACK:
		/* ntsout -> black screen    */
		printk_dbg((_DEBUG_NTSHW & 0x40),
		 "<set NTSOUT On (BlackBack)>\n");
		iRet = 0;
		break;
	case NTS_NTSOUT_DISABLE:
		/* ntsout -> stop            */
		printk_dbg((_DEBUG_NTSHW & 0x40),
		 "<set NTSOUT Off>\n");
		iRet = 0;
		break;
	default:
		iRet = -EINVAL;
		break;
	}
	if (iRet == 0)
		writel(iSetNtsoutFlg, ntshw->NTSCMmioV + NTS_OUT);

	return iRet;
}


/******************************************************************************
* MODULE   : ntshw_set_control_upscale
* FUNCTION : change NTS_CONTROL upscale register
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
#ifdef NOT_USE_FUNC
static int ntshw_set_control_upscale(int iSetUpscaleFlg)
{
	int iRet;
	unsigned long ulRegVal32;

	printk_dbg((_DEBUG_NTSHW & 0x01), "(%d)\n", iSetUpscaleFlg);

	switch (iSetUpscaleFlg) {
	case NTS_UPSCALE_ON:	/* upscale -> ON       FALL THROUGH */
	case NTS_UPSCALE_OFF:	/* upscale -> OFF */
		ulRegVal32 = readl(ntshw->NTSCMmioV + NTS_CONTROL);
		ulRegVal32 = (ulRegVal32 & ~NTS_UPSCALE_BIT) | (iSetUpscaleFlg);
		writel(ulRegVal32, ntshw->NTSCMmioV + NTS_CONTROL);
		iRet = 0;
		break;
	default:
		iRet = -EINVAL;
		break;
	}
	return iRet;
}
#endif /* NOT_USE_FUNC */


/******************************************************************************
* MODULE   : ntshw_set_control_outmode
* FUNCTION : change NTS_CONTROL outmode register
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
static int ntshw_set_control_outmode(int iSetOutmodeFlg)
{
	int iRet;
	unsigned long ulRegVal32;

	printk_dbg((_DEBUG_NTSHW & 0x01), "(%d)\n", iSetOutmodeFlg);

	switch (iSetOutmodeFlg) {
	case NTS_OUTMODE_PAL:	/* outmode -> PAL       FALL THROUGH */
	case NTS_OUTMODE_NTSC:	/* outmode -> NTSC */
		ulRegVal32 = readl(ntshw->NTSCMmioV + NTS_CONTROL);
		ulRegVal32 = (ulRegVal32 & ~NTS_OUTMODE_BIT) | (iSetOutmodeFlg);
		writel(ulRegVal32, ntshw->NTSCMmioV + NTS_CONTROL);
		ntshw->Outmode =
			iSetOutmodeFlg ? NTS_OUTPUT_PAL : NTS_OUTPUT_NTSC;
		iRet = 0;
		break;
	default:
		ntshw->Outmode = NTS_OUTPUT_DISABLE;
		iRet = -EINVAL;
		break;
	}
	return iRet;
}


/******************************************************************************
* MODULE   : ntshw_set_areaad
* FUNCTION : change NTS_AREAAD register
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
static int ntshw_set_areaad(int iSetAreaFlg, unsigned long ulSetAddrY,
 unsigned long ulSetAddrUV)
{
	int iRet;
	printk_dbg((_DEBUG_NTSHW & 0x01), "(%d) Y(%08lx) UV(%08lx)\n",
	 iSetAreaFlg, ulSetAddrY, ulSetAddrUV);

	switch (iSetAreaFlg) {
	case NTS_AREAAD_A:	/* framebuffer area -> A       FALL THROUGH */
	case NTS_AREAAD_B:	/* framebuffer area -> B       FALL THROUGH */
	case NTS_AREAAD_C:	/* framebuffer area -> C */
		writel(ulSetAddrY,  ntshw->NTSCMmioV + NTS_YAREAAD_A
		 + iSetAreaFlg);
		writel(ulSetAddrUV, ntshw->NTSCMmioV + NTS_UVAREAAD_A
		 + iSetAreaFlg);
		iRet = 0;
		break;
	default:
		iRet = -EINVAL;
		break;
	}
	return iRet;
}


/******************************************************************************
* MODULE   : ntshw_set_hoffset
* FUNCTION : change NTS_HOFFSET register
* RETURN   : 0     : success
* NOTE     : none
******************************************************************************/
static int ntshw_set_hoffset(unsigned long ulSetOffset)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "(%lx)\n", ulSetOffset);

	writel(ulSetOffset,  ntshw->NTSCMmioV + NTS_HOFFSET);
	return 0;
}


/******************************************************************************
* MODULE   : ntshw_set_register
* FUNCTION : register initialize & restore
* RETURN   :
* NOTE     : none
******************************************************************************/
static int ntshw_set_register(int iSetOutmode)
{
	int iRet;

	/* disable NTSC interrupts      */
	ntshw_set_interruput(NTSHW_INTFFCLR);
	ntshw_set_interruput(NTSHW_INTENCLR);

	/* set AreaAddress              */
	if (ntshw_set_areaad(NTS_AREAAD_A, ntshw->NtscFrameA_Y,
	 ntshw->NtscFrameA_UV)) {
		iRet = -EINVAL;
		goto fail_areaad_a;
	}
	if (ntshw_set_areaad(NTS_AREAAD_B, ntshw->NtscFrameB_Y,
	 ntshw->NtscFrameB_UV)) {
		iRet = -EINVAL;
		goto fail_areaad_b;
	}
	if (ntshw_set_areaad(NTS_AREAAD_C, ntshw->NtscFrameC_Y,
	 ntshw->NtscFrameC_UV)) {
		iRet = -EINVAL;
		goto fail_areaad_c;
	}

	/* set HOFFSET                  */
	if (ntshw_set_hoffset(NTS_WIDTH)) {
		iRet = -EINVAL;
		goto fail_hoffset;
	}

	/* set output mode              */
	switch (iSetOutmode) {
	case NTS_OUTPUT_NTSC:
		iRet = ntshw_set_control_outmode(NTS_OUTMODE_NTSC);
		break;
	case NTS_OUTPUT_PAL:
		iRet = ntshw_set_control_outmode(NTS_OUTMODE_PAL);
		break;
	default:
		iRet = -EINVAL;
		goto fail_outputmode;
	}
	goto success;

fail_outputmode:
	ntshw_set_hoffset(0);
fail_hoffset:
	ntshw_set_areaad(NTS_AREAAD_C, 0, 0);
fail_areaad_c:
	ntshw_set_areaad(NTS_AREAAD_B, 0, 0);
fail_areaad_b:
	ntshw_set_areaad(NTS_AREAAD_A, 0, 0);
fail_areaad_a:
success:
	return iRet;
}


/* ------------------ NTSC rgister check function -------------------------- */
/******************************************************************************
* MODULE   : ntshw_chk_control
* FUNCTION : check NTSC control
* RETURN   :
* NOTE     : none
******************************************************************************/
unsigned long ntshw_chk_control(void)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");
	/* check nts control */
	return (unsigned long)readl(ntshw->NTSCMmioV + NTS_CONTROL);
}


/******************************************************************************
* MODULE   : ntshw_chk_status
* FUNCTION : check NTSC status
* RETURN   :
* NOTE     : none
******************************************************************************/
unsigned long ntshw_chk_status(void)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");
	/* check nts status */
	return (unsigned long)readl(ntshw->NTSCMmioV + NTS_STATUS);
}


/*****************************************************************************
* MODULE   : ntshw_chk_framesel
* FUNCTION : check NTSC framesel
* RETURN   :
* NOTE     : none
******************************************************************************/
unsigned long ntshw_chk_framesel(void)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");
	/*  check framesel */
	return (unsigned long)readl(ntshw->NTSCMmioV + NTS_FRAMESEL);
}


/*****************************************************************************
* MODULE   : ntshw_chk_intstatus
* FUNCTION : check NTSC interrupt status
* RETURN   :
* NOTE     : none
******************************************************************************/
unsigned long ntshw_chk_intstatus(void)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");
	/*  check Interuppt Status */
	return (unsigned long)readl(ntshw->NTSCMmioV + NTS_INTSTATUS);
}


/*****************************************************************************
* MODULE   : ntshw_chk_intrawstatus
* FUNCTION : check NTSC interrupt RAW status
* RETURN   :
* NOTE     : none
******************************************************************************/
#ifdef NOT_USE_FUNC
static unsigned long ntshw_chk_intrawstatus(void)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");
	/*  check Interuppt RAW Status */
	return (unsigned long)readl(ntshw->NTSCMmioV + NTS_INTRAWSTATUS);
}
#endif /* NOT_USE_FUNC */


/******************************************************************************
* MODULE   : ntshw_save_reg
* FUNCTION : save NTS(H/W) register data
* RETURN   : none
* NOTE     : none
******************************************************************************/
void ntshw_save_reg(void)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");
}


/******************************************************************************
* MODULE   : ntshw_restore_reg
* FUNCTION : restore NTS(H/W) register data
* RETURN   : none
* NOTE     : none
******************************************************************************/
void ntshw_restore_reg(void)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");
	ntshw_set_register(ntshw->Outmode);
}


/* ------------------ NTSC initialize function ------------------------------ */
/******************************************************************************
* MODULE   : ntshw_reserve
* FUNCTION : NTS(H/W) initialized
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
int ntshw_reserve(int iSetOutmode)
{
	int iRet = -1;
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");

	/********************************/
	/* port / terminal switching    */
	/********************************/
#ifdef CONFIG_MACH_EMEV
	writel((readl(CHG_PINSEL_NTSC) & ~CHG_PINSEL_NTSC_BIT)
	 | CHG_PINSEL_NTSC_INIT, CHG_PINSEL_NTSC);
#endif
#ifdef CONFIG_MACH_EMGR
	writel((readl(CHG_PINSEL_AB) & ~CHG_PINSEL_AB_BIT)
	 | CHG_PINSEL_AB_INIT, CHG_PINSEL_AB);
	writel(0, CHG_BUSHOLD);
	writel((readl(CHG_PULL11) & ~0x00000F00) | 0x00000400, CHG_PULL11);
	writel((readl(CHG_PULL12) & ~0x44444400), CHG_PULL12);
	writel((readl(CHG_PULL13) & ~0x00000044), CHG_PULL13);
#endif

	/********************************/
	/* UnReset Module               */
	/********************************/
	ntshw_module_reset(NTSHW_UNRESET);

	/********************************/
	/* UnReset Device               */
	/********************************/
	ntshw_reset(NTSHW_UNRESET);

	/********************************/
	/* Set NTSC register             */
	/********************************/
	iRet = ntshw_set_register(iSetOutmode);

	/********************************/
	/* clear frame buffer           */
	/********************************/
	ntshw_clr_framebuf(NTS_DISP_FRAME_A);
	ntshw_clr_framebuf(NTS_DISP_FRAME_B);

	return iRet;
}


/******************************************************************************
* MODULE   : ntshw_release
* FUNCTION : NTS(H/W) initialized
* RETURN   : none
* NOTE     : none
******************************************************************************/
void ntshw_release(void)
{
	/********************************/
	/* Set NTSC register             */
	/********************************/
	ntshw_set_hoffset(0);
	ntshw_set_areaad(NTS_AREAAD_A, 0, 0);
	ntshw_set_areaad(NTS_AREAAD_B, 0, 0);
	ntshw_set_areaad(NTS_AREAAD_C, 0, 0);

	/********************************/
	/* Reset Module/Device          */
	/********************************/
	ntshw_reset(NTSHW_RESET);
	ntshw_module_reset(NTSHW_RESET);
}


/******************************************************************************
* MODULE   : ntshw_clr_framebuf
* FUNCTION : NTS(H/W) initialized
* RETURN   : none
* NOTE     : none
******************************************************************************/
void ntshw_clr_framebuf(int iClrFrameNo)
{
	struct image_data buffdata;
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");

	buffdata.uiFormat      = SIZ_FORMAT_YUV422SP;
	buffdata.uiX           = 0;
	buffdata.uiY           = 0;
	buffdata.uiWidth       = NTS_WIDTH;
	buffdata.uiHeight      = NTS_HEIGHT;
	buffdata.uiScreenWidth = NTS_WIDTH;

	if (iClrFrameNo == NTS_DISP_FRAME_A) {
		buffdata.ulPhysAddrYRGB = ntshw->NtscFrameA_Y;
		buffdata.ulPhysAddrUV   = ntshw->NtscFrameA_UV;
	} else { /* NTS_DISP_FRAME_B */
		buffdata.ulPhysAddrYRGB = ntshw->NtscFrameB_Y;
		buffdata.ulPhysAddrUV   = ntshw->NtscFrameB_UV;
	}
	buffdata.ulPhysAddrV = 0;

	/* memory clear */
	nts_image_clr_framebuf(&buffdata);
}


/******************************************************************************
* MODULE   : ntshw_init_framebuf
* FUNCTION : NTS(H/W) initialized
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
static int ntshw_init_framebuf(struct emxx_nts_dev *ntsc)
{
	int iRet = 0;

	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");

	ntshw->SmemV = ioremap_nocache(ntshw->Smem, ntshw->SmemLength);
	if (!ntshw->SmemV) {
		printk_wrn("cannot ioremap_nocache smem\n");
		iRet = -ENOMEM;
		goto fail_ioremap;
	}
	ntshw->NtscFrameAV = ntshw->SmemV + NTSCFRAME_A_OFFSET;
	ntshw->NtscFrameBV = ntshw->SmemV + NTSCFRAME_B_OFFSET;
	printk_dbg((_DEBUG_NTSHW),
	 "NtscFrameA(0x%lx)  NtscFrameAV(0x%p)  NtscFrameLength (0x%lx)\n",
	 ntshw->NtscFrameA_Y, ntshw->NtscFrameAV, ntshw->NtscFrameLength);
	printk_dbg((_DEBUG_NTSHW),
	 "NtscFrameB(0x%lx)  NtscFrameBV(0x%p)  NtscFrameLength (0x%lx)\n",
	 ntshw->NtscFrameB_Y, ntshw->NtscFrameBV, ntshw->NtscFrameLength);

	ntsc->Smem = ntshw->Smem;
	ntsc->SmemLength = ntshw->SmemLength;
	ntsc->SmemV = ntshw->SmemV;

	iRet = 0;
	goto success;

fail_ioremap:
success:
	return iRet;
}


/******************************************************************************
* MODULE   : ntshw_exit_framebuf
* FUNCTION : NTS(H/W) initialized
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void ntshw_exit_framebuf(void)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");

	ntshw_set_areaad(NTS_AREAAD_A, 0, 0);
	ntshw_set_areaad(NTS_AREAAD_B, 0, 0);
	ntshw_set_areaad(NTS_AREAAD_C, 0, 0);
	ntshw_set_hoffset(0);

	if (ntshw->NtscFrameAV)
		iounmap(ntshw->NtscFrameAV);
	if (ntshw->NtscFrameBV)
		iounmap(ntshw->NtscFrameBV);
	if (ntshw->SmemV)
		iounmap(ntshw->SmemV);

	ntshw->NtscFrameAV = NULL;
	ntshw->NtscFrameBV = NULL;
	ntshw->SmemV     = NULL;
}


/******************************************************************************
* MODULE   : ntshw_init_val
* FUNCTION : NTS(H/W) initialized
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void *ntshw_init_val(void)
{
	void *alloc_val;

	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");

	alloc_val = kmalloc(sizeof(struct emxx_ntshw), GFP_KERNEL);
	if (!alloc_val) {
		;
	} else {
		memset(alloc_val, 0, sizeof(struct emxx_ntshw));

		/* NTSC MMIO */
		((struct emxx_ntshw *)alloc_val)->NTSCMmio        =
			EMXX_NTS_BASE;
		((struct emxx_ntshw *)alloc_val)->NTSCMmioV       = NULL;

		/* NTSC framebuffer */
		((struct emxx_ntshw *)alloc_val)->NtscFrameA_Y    =
			NTSC_SMEM_START + NTSCFRAME_A_OFFSET;
		((struct emxx_ntshw *)alloc_val)->NtscFrameA_UV   =
			((struct emxx_ntshw *)alloc_val)->NtscFrameA_Y +
			NTS_WIDTH * NTS_HEIGHT;
		((struct emxx_ntshw *)alloc_val)->NtscFrameB_Y    =
			NTSC_SMEM_START + NTSCFRAME_B_OFFSET;
		((struct emxx_ntshw *)alloc_val)->NtscFrameB_UV   =
			((struct emxx_ntshw *)alloc_val)->NtscFrameB_Y +
			NTS_WIDTH * NTS_HEIGHT;
		((struct emxx_ntshw *)alloc_val)->NtscFrameC_Y    = 0l;
		((struct emxx_ntshw *)alloc_val)->NtscFrameC_UV   = 0l;
		((struct emxx_ntshw *)alloc_val)->NtscFrameLength =
			NTSCFRAME_LENGTH;
		((struct emxx_ntshw *)alloc_val)->NtscFrameBV     = NULL;
		((struct emxx_ntshw *)alloc_val)->NtscFrameAV     = NULL;

		/* Smem (common memory to LCD) */
		((struct emxx_ntshw *)alloc_val)->Smem            =
			NTSC_SMEM_START;
		((struct emxx_ntshw *)alloc_val)->SmemLength      =
			NTSC_SMEM_LENGTH;
		((struct emxx_ntshw *)alloc_val)->SmemV           = NULL;
	}
	return alloc_val;
}


/******************************************************************************
* MODULE   : ntshw_init
* FUNCTION : NTS(H/W) initialized
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
int ntshw_init(struct emxx_nts_dev *ntsc)
{
	int iRet = 0;

	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");

	ntshw = (struct emxx_ntshw *)ntshw_init_val();
	if (!ntshw) {
		printk_wrn("failed ntshw_init_val\n");
		goto fail_alloc_val;
	}

	/* reserve NTSMmio */
	ntshw->NTSCMmioV = (char *)IO_ADDRESS(ntshw->NTSCMmio);
	printk_dbg((_DEBUG_NTSHW),
	 "NTSCMmio(0x%08lx)  NTSCMmioV(0x%p)\n",
	 ntshw->NTSCMmio, ntshw->NTSCMmioV);

	/* init framebuffer */
	if (ntshw_init_framebuf(ntsc)) {
		printk_wrn("failed ntshw_init_framebuf\n");
		goto fail_init_framebuf;
	}
	goto success;

fail_init_framebuf:
	kfree(ntshw);	/* NULL check is inside. */
fail_alloc_val:
	iRet = -ENOMEM;

success:
	return iRet;
}


/******************************************************************************
* MODULE   : ntshw_exit
* FUNCTION : NTS(H/W) initialized
* RETURN   : none
* NOTE     : none
******************************************************************************/
void ntshw_exit(void)
{
	printk_dbg((_DEBUG_NTSHW & 0x01), "\n");

	/* disable NTS_OUT */
	ntshw_start(NTSHW_STOP);

	/* disable NTSC interrupts */
	ntshw_set_interruput(NTSHW_INTFFCLR);
	ntshw_set_interruput(NTSHW_INTENCLR);

	/* Reset Module/Device */
	ntshw_reset(NTSHW_RESET);
	ntshw_module_reset(NTSHW_RESET);

	if (ntshw) {
		ntshw_exit_framebuf();
		memset(ntshw, 0, sizeof(struct emxx_ntshw));
		kfree(ntshw);
	}
	ntshw = NULL;
}


