/*
 *  File Name		: linux/drivers/mmc/host/emxx_sdio.c
 *  Function		: MMC
 *  Release Version	: Ver 1.08
 *  Release Date	: 2011/02/16
 *
 *  Copyright (C) 2010-2011 Renesas Electronics Corporation
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/smu.h>
#include <mach/pmu.h>

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#include <mach/pm.h>
#endif

/* #define MMC_USE_SDMA */
/* #define MMC_USE_CPUSEND */

#include "emxx_sdio.h"

#define DRIVER_NAME	"emxx_sdio"
#define DRIVER_NAME2	"emxx_sdio1"

#define MMC_CLK_ON	emxx_open_clockgate(host->clk | host->sclk)
#define MMC_CLK_OFF	emxx_close_clockgate(host->clk | host->sclk)

/* #define MMC_SPEED_NORMAL */

/* #define DEBUG_PRINT */
/* #define DEBUG_ERR_PRINT */
#ifdef DEBUG_PRINT
#define FUNC_PRINT(FMT, ARGS...)	\
		printk(KERN_INFO "%s: %s(): " FMT,\
			mmc_hostname(host->mmc), __func__, ##ARGS)
#define FUNC_ERR_PRINT(FMT, ARGS...)	\
		printk(KERN_INFO "%s: %s(): " FMT,\
			mmc_hostname(host->mmc), __func__, ##ARGS)
#else
#define FUNC_PRINT(FMT, ARGS...)
#ifdef DEBUG_ERR_PRINT
#define FUNC_ERR_PRINT(FMT, ARGS...)	\
		printk(KERN_INFO "%s: %s(): " FMT,\
			mmc_hostname(host->mmc), __func__, ##ARGS)
#else
#define FUNC_ERR_PRINT(FMT, ARGS...)
#endif
#endif

static struct emxx_mmc_host *g_host[3];

static void emxx_sdio_reset(struct emxx_mmc_host *host, u32 mask)
{
	u32 val;

	val = readl(host->base + SDIO_CLK_TOUT_RST);
	writel(val | mask, host->base + SDIO_CLK_TOUT_RST);

	while (readl(host->base + SDIO_CLK_TOUT_RST) & mask)
		udelay(10);
}

/*
 * Notify the core about command completion
 */
static void
emxx_sdio_data_done(struct emxx_mmc_host *host, struct mmc_command *cmd)
{
	struct mmc_data *data = host->data;
	u32 val;

	if (data != NULL) {
#ifndef MMC_USE_CPUSEND
		dma_unmap_sg(mmc_dev(host->mmc), data->sg,
						host->dma_num, host->dma_dir);
#endif

		if (data->error) {
			emxx_sdio_reset(host,
				SDIO_SOFTRST_CMD | SDIO_SOFTRST_DATA);
			data->bytes_xfered = 0;
		} else
#ifdef MMC_USE_CPUSEND
			data->bytes_xfered = data->blksz * data->blocks;
#else
			data->bytes_xfered = host->data_len;
#endif

		val = readl(host->base + SDIO_INT_STSEN) & ~SDIO_INT_TRANCOMP;
		writel(val, host->base + SDIO_INT_STSEN);

		MMC_CLK_OFF;

		host->data_len = 0;

		host->data = NULL;
		host->cmd = NULL;
		host->req = NULL;
		host->stop = NULL;

		mmc_request_done(host->mmc, cmd->mrq);
	}
}

/*
 * Notify the core about command completion
 */
static void
emxx_sdio_cmd_done(struct emxx_mmc_host *host, struct mmc_command *cmd)
{
	u32 val;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			val = readl(host->base + SDIO_RSP01);
			cmd->resp[3] = (val & 0x00ffffff) << 8;
			cmd->resp[2] = (val & 0xff000000) >> 24;
			val = readl(host->base + SDIO_RSP23);
			cmd->resp[2] |= (val & 0x00ffffff) << 8;
			cmd->resp[1] = (val & 0xff000000) >> 24;
			val = readl(host->base + SDIO_RSP45);
			cmd->resp[1] |= (val & 0x00ffffff) << 8;
			cmd->resp[0] = (val & 0xff000000) >> 24;
			val = readl(host->base + SDIO_RSP67);
			cmd->resp[0] |= (val & 0x00ffffff) << 8;
			FUNC_PRINT("CMD=%d RSP %x %x %x %x\n",
				cmd->opcode, cmd->resp[0],
				cmd->resp[1], cmd->resp[2], cmd->resp[3]);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = readl(host->base + SDIO_RSP01);
			FUNC_PRINT("CMD=%d RSP %x\n",
				cmd->opcode, cmd->resp[0]);
		}
	}

	if (host->data == NULL) {
		if (cmd->error)
			emxx_sdio_reset(host,
				SDIO_SOFTRST_CMD | SDIO_SOFTRST_DATA);
		if ((cmd->flags & MMC_RSP_BUSY) == 0)
			MMC_CLK_OFF;
		host->req = NULL;
		host->cmd = NULL;
		mmc_request_done(host->mmc, cmd->mrq);
	} else if (host->data->error)
		emxx_sdio_data_done(host, host->cmd);
}

/*
 * Configure the Responce type
 */
static void
emxx_sdio_start_cmd(struct emxx_mmc_host *host,
			struct mmc_command *cmd, u32 cmddat)
{
	u32 resp = mmc_resp_type(cmd);
	u32 mask;
	unsigned long timeout;

	mask = SDIO_STATE_CMD_INHIBIT;
	if ((cmd->data != NULL) || (cmd->flags & MMC_RSP_BUSY))
		mask |= SDIO_STATE_DAT_INHIBIT;

	/* Wait max 10 ms */
	timeout = 10;

	while (readl(host->base + SDIO_STATE) & mask) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Controller never released "
				"inhibit bit(s).\n", mmc_hostname(host->mmc));
			emxx_sdio_reset(host,
				SDIO_SOFTRST_CMD | SDIO_SOFTRST_DATA);
			MMC_CLK_OFF;
			mmc_request_done(host->mmc, cmd->mrq);
			return;
		}
		timeout--;
		mdelay(1);
	}

	/* Clear interrupt */
	writel(0xffffffff, host->base + SDIO_INT_STS);

	host->cmd = cmd;

	cmddat |= SDIO_CMD_INDEX(cmd->opcode);

	if (resp & MMC_RSP_CRC)
		cmddat |= SDIO_CMD_CRC_CHK;
	if (resp & MMC_RSP_OPCODE)
		cmddat |= SDIO_CMD_INDEX_CHK;

	if (resp & MMC_RSP_136)
		cmddat |= SDIO_CMD_RESP_136;
	else if (resp & MMC_RSP_BUSY)
		cmddat |= SDIO_CMD_RESP_48B;
	else if (resp & MMC_RSP_PRESENT)
		cmddat |= SDIO_CMD_RESP_48;

	writel(cmd->arg, host->base + SDIO_ARG);

	FUNC_PRINT("CMD=%d arg=0x%x flags=0x%x retry=%d reg=0x%08x\n",
		cmd->opcode, cmd->arg, cmd->flags, cmd->retries, cmddat);

	/* Send command */
	writel(cmddat, host->base + SDIO_MODE_CMD);
}

static void emxx_sdio_set_dma(struct emxx_mmc_host *host)
{
#ifdef MMC_USE_SDMA
	if (host->dma_count < host->dma_num) {
		writel(host->dma_address[host->dma_count++],
			host->base + SDIO_SYSADD);
	}
#endif
}

static void emxx_sdio_cpu_datasend(struct emxx_mmc_host *host)
{
#ifdef MMC_USE_CPUSEND
	int i, j;
	u32 val;
	u32 *data = (u32 *)(host->dma_address[0]);
	u32 mask, len = 0, addr_num = 0;

	if (host->data->flags & MMC_DATA_READ)
		mask = SDIO_STATE_RDEN;
	else
		mask = SDIO_STATE_WREN;

	if (host->data_len == 0)
		return;

	for (j = 0; j < host->data->blocks; j++) {
		while ((readl(host->base + SDIO_STATE) & mask) == 0)
			udelay(10);

		for (i = 0; i < host->data->blksz / sizeof(u32); i++) {
			if (host->data->flags & MMC_DATA_READ) {
				val = readl(host->base + SDIO_BUF);
				*data = val;
			} else {
				writel(*data, host->base + SDIO_BUF);
			}
			data++;
		}
		len += host->data->blksz;
		if (host->dma_len[addr_num] == len) {
			addr_num++;
			len = 0;
			data = (u32 *)(host->dma_address[addr_num]);
		}
	}

	host->data_len = 0;
#endif
}

/*
 * MMC controller IRQ handler
 */
static irqreturn_t
emxx_sdio_irq(int irq, void *dev_id)
{
	struct emxx_mmc_host *host = (struct emxx_mmc_host *)dev_id;
	u32 status, end_trans = 0;

	MMC_CLK_ON;

	/* Read interrupt */
	status = readl(host->base + SDIO_INT_STS);
	/* Clear interrupt */
	writel(status, host->base + SDIO_INT_STS);

	if (!(status & SDIO_INT_SDIO_INT) && (host->data == NULL)) {
		if (host->cmd == NULL) {
			MMC_CLK_OFF;
			return IRQ_HANDLED;
		}
	}

	if (status & SDIO_INT_ALLERR) {
		end_trans = 1;
		if (status & SDIO_INT_CMD_TOUT) {
			FUNC_ERR_PRINT("CMD=%d CMD Timeout\n",
					host->cmd->opcode);
			host->cmd->error = -ETIMEDOUT;
		} else if (status & SDIO_INT_CMD_CRC) {
			FUNC_ERR_PRINT("CMD=%d CMD CRC error\n",
					host->cmd->opcode);
			host->cmd->error = -EILSEQ;
		} else {
			FUNC_ERR_PRINT("CMD=%d Other error.(0x%x)\n",
					host->cmd->opcode, status);
			host->cmd->error = -EILSEQ;
		}
		if (host->data) {
			if (status & SDIO_INT_DATA_TOUT) {
				FUNC_ERR_PRINT("CMD=%d DATA Timeout\n",
						host->cmd->opcode);
				host->data->error = -ETIMEDOUT;
			} else if (status & SDIO_INT_DATA_CRC) {
				FUNC_ERR_PRINT("CMD=%d DATA CRC error\n",
						host->cmd->opcode);
				host->data->error = -EILSEQ;
			} else {
				FUNC_ERR_PRINT("CMD=%d Other error.(0x%x)\n",
						host->cmd->opcode, status);
				host->data->error = -EILSEQ;
			}
		}
		if (status & SDIO_INT_ADMA_ERR) {
			FUNC_ERR_PRINT("ADMA err = 0x%x\n",
				readl(host->base + SDIO_ADMA_ERR));
		}
	}

	if (status & SDIO_INT_SDIO_INT)
		mmc_signal_sdio_irq(host->mmc);

	if ((status & SDIO_INT_CMDCOMP) || end_trans)
		emxx_sdio_cmd_done(host, host->cmd);
	if (end_trans == 1)
		return IRQ_HANDLED;

	if (status & SDIO_INT_DMA)
		emxx_sdio_set_dma(host);

	if (status & (SDIO_INT_RREADY | SDIO_INT_WREADY))
		emxx_sdio_cpu_datasend(host);

	if (status & SDIO_INT_TRANCOMP)
		emxx_sdio_data_done(host, host->cmd);

	return IRQ_HANDLED;
}

/*
 * IRQ for handling card insertion and removal
 */
static irqreturn_t
emxx_sdio_detect_irq(int irq, void *dev_id)
{
	struct emxx_mmc_host *host = (struct emxx_mmc_host *)dev_id;
	u32 val;

	FUNC_PRINT("\n");

	val = gpio_get_value(host->detect_gpio);
	if (val) {
		host->connect = 0;
		set_irq_type(host->detect_irq, IRQ_TYPE_LEVEL_LOW);
	} else {
		host->connect = 1;
		set_irq_type(host->detect_irq, IRQ_TYPE_LEVEL_HIGH);
	}
	mmc_detect_change(host->mmc, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

/*
 * Configure block leangth for MMC/SD cards and intiate the transfer.
 */
static void
emxx_sdio_setup_data(struct emxx_mmc_host *host, struct mmc_data *data)
{
	int i;

#if !defined(MMC_USE_SDMA) && !defined(MMC_USE_CPUSEND)
	u32 tmp_addr, tmp_len, offset;
	u8 *desc = host->adma_virt_addr;
	u8 *align = host->adma_align_virt_addr;
	dma_addr_t align_addr = host->adma_align_addr;
	u8 *buffer;
#endif

	host->data = data;
	host->data_len = data->blksz * data->blocks;

	FUNC_PRINT("size=%d num=%d flags=0x%x\n",
				data->blksz, data->blocks, data->flags);

	host->dma_dir = (data->flags & MMC_DATA_WRITE) ?
				DMA_TO_DEVICE : DMA_FROM_DEVICE;
	host->dma_num = dma_map_sg(mmc_dev(host->mmc), data->sg,
				data->sg_len, host->dma_dir);

	writel(SDIO_BLK_LENGTH(data->blksz) | SDIO_BLK_BOUND_512K |
		SDIO_BLK_COUNT(data->blocks),  host->base + SDIO_BLOCK);

#if defined(MMC_USE_CPUSEND)
	for (i = 0; i < host->dma_num; i++) {
		host->dma_address[i] =
			(u32)phys_to_virt(sg_dma_address(&data->sg[i]));
		host->dma_len[i] = sg_dma_len(&data->sg[i]);
	}
#elif defined(MMC_USE_SDMA)
	for (i = 0; i < host->dma_num; i++)
		host->dma_address[i] = sg_dma_address(&data->sg[i]);

	host->dma_count = 0;

	emxx_sdio_set_dma(host);
#else
	for (i = 0; i < host->dma_num; i++) {
		tmp_addr = sg_dma_address(&data->sg[i]);
		tmp_len = sg_dma_len(&data->sg[i]);
		offset = (4 - (tmp_addr & 0x3)) & 0x3;
		if (offset) {
			if (data->flags & MMC_DATA_WRITE) {
				buffer = (u8 *)phys_to_virt(tmp_addr);
				memcpy(align, buffer, offset);
			}

			desc[7] = (align_addr >> 24) & 0xff;
			desc[6] = (align_addr >> 16) & 0xff;
			desc[5] = (align_addr >> 8) & 0xff;
			desc[4] = (align_addr >> 0) & 0xff;

			BUG_ON(offset > 65536);

			desc[3] = (offset >> 8) & 0xff;
			desc[2] = (offset >> 0) & 0xff;
			desc[1] = 0x00;
			desc[0] = 0x21; /* tran, valid */

			align += 4;
			align_addr += 4;

			desc += 8;

			tmp_addr += offset;
			tmp_len -= offset;
		}

		desc[7] = (tmp_addr >> 24) & 0xff;
		desc[6] = (tmp_addr >> 16) & 0xff;
		desc[5] = (tmp_addr >> 8) & 0xff;
		desc[4] = (tmp_addr >> 0) & 0xff;
		desc[3] = (tmp_len >> 8) & 0xff;
		desc[2] = (tmp_len >> 0) & 0xff;
		desc[1] = 0x00;
		desc[0] = 0x21; /* tran, valid */

		desc += 8;
	}

	host->dma_count = 0;

	desc -= 8;
	desc[0] = 0x23; /* tran, end, valid */
	writel(host->adma_addr, host->base + SDIO_ADMA_SYSADD);
#endif
}

/*
 * Request function. for read/write operation
 */
static void
emxx_sdio_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct emxx_mmc_host *host = mmc_priv(mmc);
	u32 cmddat = 0;
	u32 val;

	if (host->connect == 0) {
		req->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, req);
		return;
	}

	host->req = req;
	host->stop = req->stop;

	MMC_CLK_ON;

	if (host->idle_suspend == 1) {
		val = readl(host->base + SDIO_GIO0);
		writel(val | SDIO_GIO0_DETECT, host->base + SDIO_GIO0);
		writel(val & ~SDIO_GIO0_DETECT, host->base + SDIO_GIO0);
		udelay(200);
		val = readl(host->base + SDIO_CLK_TOUT_RST);
		writel(val | SDIO_CLK_SDCLKEN, host->base + SDIO_CLK_TOUT_RST);
		host->idle_suspend = 0;
	}

	if (req->data) {
		emxx_sdio_setup_data(host, req->data);

#ifdef MMC_USE_CPUSEND
		cmddat |= SDIO_CMD_DATA | SDIO_MODE_BLK_COUNT_EN;
#else
		cmddat |= SDIO_CMD_DATA |
			SDIO_MODE_DMA_EN | SDIO_MODE_BLK_COUNT_EN;
#endif
		if (host->data->flags & MMC_DATA_READ)
			cmddat |= SDIO_MODE_READ;

		if (host->data->blocks > 1)
			cmddat |= SDIO_MODE_MULTI;

		val = readl(host->base + SDIO_INT_STSEN) | SDIO_INT_TRANCOMP;
		writel(val, host->base + SDIO_INT_STSEN);
	}
	if (host->stop)
		cmddat |= SDIO_MODE_ACMD12;

	emxx_sdio_start_cmd(host, req->cmd, cmddat);
}

static void
emxx_sdio_init_hw(struct emxx_mmc_host *host)
{
	u32 val;

	MMC_CLK_ON;
	emxx_unreset_device(host->reset);

	writel(SDIO_MODEN_ENABLE, host->base + SDIO_MODEN);
	writel(SDIO_DELAY_REVERSE, host->base + SDIO_DELAY);

	val = readl(host->base + SDIO_GIO0) & ~SDIO_GIO0_DETECT;
#ifdef CONFIG_MACH_EMGR
	val |= SDIO_GIO0_DELAYSEL;
#endif
	writel(val, host->base + SDIO_GIO0);

	udelay(200);

	emxx_sdio_reset(host, SDIO_SOFTRST_ALL);

	/* set response timeout count */
	writel(SDIO_TIMEOUT_COUNT_MAX, host->base + SDIO_CLK_TOUT_RST);

	/* Unmask interrupts */
	writel(SDIO_INT_MASK, host->base + SDIO_INT_STSEN);
	writel(SDIO_INT_MASK | SDIO_INT_TRANCOMP,
			host->base + SDIO_INT_SIGEN);

	val = readl(host->base + SDIO_GIO1) | SDIO_GIO1_INTSEL;
	writel(val, host->base + SDIO_GIO1);

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1)
		writel(SDIO_AMBA0_TMODE_SINGLE, host->base + SDIO_AMBA0);
	else
#endif
		writel(SDIO_AMBA0_TMODE_INCR16, host->base + SDIO_AMBA0);
	MMC_CLK_OFF;

	val = readl(host->pin_sel);
	writel(val & ~host->pin_cko, host->pin_sel);
}

static void emxx_sdio_power(struct emxx_mmc_host *host, u32 power_on)
{
	u32 val;

	if (power_on)
		emxx_sdio_init_hw(host);

	MMC_CLK_ON;
	val = readl(host->base + SDIO_HP_BW);

	if (power_on) {
		FUNC_PRINT("on\n");
		val |= SDIO_POWER_VOLT_30 | SDIO_POWER_POWER |
#ifdef MMC_USE_SDMA
			SDIO_HOST_SDMA;
#else
			SDIO_HOST_ADMA32;
#endif
	} else {
		FUNC_PRINT("off\n");
		val = 0;
	}

	writel(val, host->base + SDIO_HP_BW);

	if (!power_on) {
		val = readl(host->pin_sel);
		writel(val | host->pin_cko, host->pin_sel);
		emxx_reset_device(host->reset);
	}
	MMC_CLK_OFF;
}

/*
 * Configuring clock values.
 */
static void
emxx_sdio_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct emxx_mmc_host *host = mmc_priv(mmc);
	u32 val, val2, div;

	FUNC_PRINT("clock=%d power=0x%x width=%d timing=%d\n",
		ios->clock, ios->power_mode, ios->bus_width, ios->timing);

	if (host->power_mode != ios->power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_UP:
			emxx_sdio_power(host, 1);
			FUNC_PRINT("power up\n");
			host->power_mode = ios->power_mode;
			break;
		case MMC_POWER_ON:
			FUNC_PRINT("power on\n");
			host->power_mode = ios->power_mode;
			break;
		}
	}

	if ((host->bus_width != ios->bus_width)
				&& (host->power_mode != MMC_POWER_OFF)) {
		MMC_CLK_ON;
		val = readl(host->base + SDIO_HP_BW) &
			~(SDIO_HOST_MMC8B | SDIO_HOST_WIDTH);
		switch (ios->bus_width) {
		case MMC_BUS_WIDTH_8:
			val |= SDIO_HOST_MMC8B;
			break;
		case MMC_BUS_WIDTH_4:
			val |= SDIO_HOST_WIDTH;
			break;
		}
		writel(val, host->base + SDIO_HP_BW);
		MMC_CLK_OFF;
	}

	if (ios->timing != host->timing) {
		FUNC_PRINT("timing = %d\n", ios->timing);
		MMC_CLK_ON;
		val  = readl(host->base + SDIO_GIO0);
		val2 = readl(host->base + SDIO_HP_BW);
		if ((ios->timing == MMC_TIMING_SD_HS)
			 || (ios->timing == MMC_TIMING_MMC_HS)) {
			val  |= SDIO_GIO0_HSENA;
			val2 |= SDIO_HOST_HS;
		} else {
			val  &= ~SDIO_GIO0_HSENA;
			val2 &= ~SDIO_HOST_HS;
		}
		FUNC_PRINT("gio0 = %x, hp_bs = %x\n", val, val2);
		writel(val,  host->base + SDIO_GIO0);
		writel(val2, host->base + SDIO_HP_BW);
		MMC_CLK_OFF;
		host->timing = ios->timing;
	}

	if ((ios->clock != host->clock)
			&& (host->power_mode != MMC_POWER_OFF)) {
		MMC_CLK_ON;
		/* Stop clock */
		val = readl(host->base + SDIO_CLK_TOUT_RST);
		val &= ~SDIO_CLK_MASK;
		writel(val, host->base + SDIO_CLK_TOUT_RST);

		if (ios->clock != 0) {
			if (ios->clock > host->base_clock)
				val |= SDIO_CLK_CLKDIV1;
			else if (ios->clock >= host->base_clock / 2)
				val |= SDIO_CLK_CLKDIV2;
			else if (ios->clock >= host->base_clock / 4)
				val |= SDIO_CLK_CLKDIV4;
			else if (ios->clock >= host->base_clock / 8)
				val |= SDIO_CLK_CLKDIV8;
			else if (ios->clock >= host->base_clock / 16)
				val |= SDIO_CLK_CLKDIV16;
			else if (ios->clock >= host->base_clock / 32)
				val |= SDIO_CLK_CLKDIV32;
			else if (ios->clock >= host->base_clock / 64)
				val |= SDIO_CLK_CLKDIV64;
			else if (ios->clock >= host->base_clock / 128)
				val |= SDIO_CLK_CLKDIV128;
			else
				val |= SDIO_CLK_CLKDIV256;

			val |= SDIO_CLK_CLKEN;

			div = readl(host->clkdiv);
			if ((ios->clock < host->base_clock / 256) &&
						(div != host->init_base)) {
				emxx_close_clockgate(host->sclk);
				writel(host->init_base, host->clkdiv);
				FUNC_PRINT("change SDIO div = 0x%x\n",
							host->init_base);
				emxx_open_clockgate(host->sclk);
			} else if ((ios->clock >= host->base_clock / 256) &&
						(div != host->normal_base)) {
				emxx_close_clockgate(host->sclk);
				writel(host->normal_base, host->clkdiv);
				FUNC_PRINT("change SDIO div = 0x%x\n",
							host->normal_base);
				emxx_open_clockgate(host->sclk);
			}

			/* Start clock */
			writel(val, host->base + SDIO_CLK_TOUT_RST);
			while (!((val = readl(host->base + SDIO_CLK_TOUT_RST))
					 & SDIO_CLK_CLKSTA))
				mdelay(1);

			val |= SDIO_CLK_SDCLKEN;
			writel(val, host->base + SDIO_CLK_TOUT_RST);
		}
		MMC_CLK_OFF;
		host->clock = ios->clock;
	}

	if (host->power_mode != ios->power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
			emxx_sdio_power(host, 0);
			FUNC_PRINT("power off\n");
			break;
		}
		host->power_mode = ios->power_mode;
	}
}

static int emxx_sdio_get_ro(struct mmc_host *mmc)
{
	struct emxx_mmc_host *host = mmc_priv(mmc);

	if (host->wp_gpio)
		return gpio_get_value(host->wp_gpio);
	else
		return 0;
}

static void emxx_sdio_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct emxx_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 stsen, sigen;

	spin_lock_irqsave(&host->lock, flags);

	MMC_CLK_ON;
	stsen = readl(host->base + SDIO_INT_STSEN);
	sigen = readl(host->base + SDIO_INT_SIGEN);
	if (enable) {
		stsen |= SDIO_INT_SDIO_INT;
		sigen |= SDIO_INT_SDIO_INT;
	} else {
		stsen &= ~SDIO_INT_SDIO_INT;
		sigen &= ~SDIO_INT_SDIO_INT;
	}
	writel(stsen, host->base + SDIO_INT_STSEN);
	writel(sigen, host->base + SDIO_INT_SIGEN);
	MMC_CLK_OFF;

	spin_unlock_irqrestore(&host->lock, flags);
}

static struct mmc_host_ops emxx_mmc_ops = {
	.request	= emxx_sdio_request,
	.set_ios	= emxx_sdio_set_ios,
	.get_ro		= emxx_sdio_get_ro,
	.enable_sdio_irq = emxx_sdio_enable_sdio_irq,
};

static int
emxx_sdio_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct emxx_mmc_host *host = NULL;
	int ret = -EINVAL;
	u32 val;

	mmc = mmc_alloc_host(sizeof(struct emxx_mmc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto err;
	}

	host      = mmc_priv(mmc);
	host->mmc = mmc;
	mmc->f_max = 0;
	mmc->caps = MMC_CAP_4_BIT_DATA;

#ifdef CONFIG_MACH_EMGR
	host->pin_sel = CHG_PINSEL_G064;
#endif

	g_host[pdev->id] = host;

	switch (pdev->id) {
	case 0:
		host->base = EMXX_MMC_SDIO0_BASE;
		host->irq = INT_SDIO0;
		host->detect_gpio = 0;
		host->detect_irq = 0;
		host->wp_gpio = 0;
		host->clk = EMXX_CLK_SDIO0_H | EMXX_CLK_SDIO0;
		host->sclk = EMXX_CLK_SDIO0_S;
		host->reset = EMXX_RST_SDIO0;
		host->clkdiv = SMU_SDIO0SCLKDIV;
		host->init_base = EMXX_MMC_SMU_DIV5;
		host->normal_base = EMXX_MMC_SMU_DIV5;
		host->base_clock = 46000000;
#ifdef CONFIG_MACH_EMEV
		host->pin_sel = CHG_PINSEL_G032;
		mmc->caps |= MMC_CAP_8_BIT_DATA;
#endif
		host->pin_cko = EMXX_CHG_SDIO0_CKO;
		mmc->card_num = 2;
		break;
	case 1:
		host->base = EMXX_MMC_SDIO1_BASE;
		host->irq = INT_SDIO1;
		host->detect_gpio = GPIO_SDI1_CD;
		host->detect_irq = INT_SDI1_CD;
		host->wp_gpio = GPIO_SDI1_WP;
		host->clk = EMXX_CLK_SDIO1_H | EMXX_CLK_SDIO1;
		host->sclk = EMXX_CLK_SDIO1_S;
		host->reset = EMXX_RST_SDIO1;
		host->clkdiv = SMU_SDIO1SCLKDIV;
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
			host->init_base = EMXX_MMC_SMU_DIV6;
			host->normal_base = EMXX_MMC_SMU_DIV3;
			host->base_clock = 76000000;
		} else {
			host->init_base = EMXX_MMC_SMU_DIV5;
			host->normal_base = EMXX_MMC_SMU_DIV5;
			host->base_clock = 46000000;
		}
		host->pin_sel = CHG_PINSEL_G032;
#else
		host->init_base = EMXX_MMC_SMU_DIV5;
		host->normal_base = EMXX_MMC_SMU_DIV5;
		host->base_clock = 46000000;
#endif
		host->pin_cko = EMXX_CHG_SDIO1_CKO;
		mmc->card_num = 1;
		break;
#ifdef CONFIG_MACH_EMEV
	case 2:
		host->base = EMXX_MMC_SDIO2_BASE;
		host->irq = INT_SDIO2;
		host->detect_gpio = GPIO_SDI2_CD;
		host->detect_irq = INT_SDI2_CD;
		host->wp_gpio = GPIO_SDI2_WP;
		host->clk = EMXX_CLK_SDIO2_H | EMXX_CLK_SDIO2;
		host->sclk = EMXX_CLK_SDIO2_S;
		host->reset = EMXX_RST_SDIO2;
		host->clkdiv = SMU_SDIO2SCLKDIV;
		host->init_base = EMXX_MMC_SMU_DIV6;
		host->normal_base = EMXX_MMC_SMU_DIV3;
		host->pin_sel = CHG_PINSEL_G096;
		host->pin_cko = EMXX_CHG_SDIO2_CKO;
		host->base_clock = 76000000;
		mmc->card_num = 1;
		mmc->f_max = 10000000;
		break;
#endif
	default:
		goto err;
	}

	val = readl(host->pin_sel);
	writel(val | host->pin_cko, host->pin_sel);
	writel(host->init_base, host->clkdiv);

	host->bus_width = 10;	/* set dummy */
	host->clock = 0;
	host->timing = 0xff;	/* set dummy */
	host->power_mode = MMC_POWER_OFF;

	mutex_init(&host->mutex);
	spin_lock_init(&host->lock);

	mmc->ops   = &emxx_mmc_ops;
	mmc->f_min = 200000;
	if (mmc->f_max == 0)
#ifdef MMC_SPEED_NORMAL
		mmc->f_max = 25000000;
#else
		mmc->f_max = 50000000;
#endif
	if (mmc->f_max > 25000000)
		mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	/* MMC core transfer sizes tunable parameters */
#ifdef MMC_USE_SDMA
	mmc->max_hw_segs   = 1;
#else
	mmc->max_hw_segs   = EMXX_MAX_SEGS;
#endif
	mmc->max_phys_segs = EMXX_MAX_SEGS;
#if defined(MMC_USE_SDMA) || defined(MMC_USE_CPUSEND)
	mmc->max_seg_size  = 524288;	/* 512KByte */
#else
	mmc->max_seg_size  = 65536;	/* 64KByte */
#endif
	mmc->max_req_size  = 524288;	/* 512KByte */

	mmc->max_blk_size  = 512;
	mmc->max_blk_count = 65535;

	host->adma_virt_addr =
		(u8 *)dma_alloc_coherent(NULL, SDIO_ADMA_BUF_SIZE,
			&host->adma_addr, GFP_KERNEL | GFP_DMA);
	if (!(host->adma_virt_addr)) {
		ret = -ENOMEM;
		goto err;
	}
	host->adma_align_virt_addr =
		(u8 *)dma_alloc_coherent(NULL, SDIO_ADMA_ALIGN_SIZE,
			&host->adma_align_addr, GFP_KERNEL | GFP_DMA);
	if (!(host->adma_align_virt_addr)) {
		ret = -ENOMEM;
		goto err_align_buf;
	}

	/* Request IRQ for MMC operations */
	ret = request_irq(host->irq, emxx_sdio_irq,
				IRQF_DISABLED, pdev->name, host);
	if (ret) {
		printk(KERN_ERR "%s: Unable to IRQ(%d)", __func__, host->irq);
		goto err_irq;
	}

	/* Request IRQ for MMC card detect */
	if (host->detect_irq && host->detect_gpio) {
		val = gpio_get_value(host->detect_gpio);
		if (val) {
			host->connect = 0;
			set_irq_type(host->detect_irq, IRQ_TYPE_LEVEL_LOW);
		} else {
			host->connect = 1;
			set_irq_type(host->detect_irq, IRQ_TYPE_LEVEL_HIGH);
		}
		ret = request_irq(host->detect_irq, emxx_sdio_detect_irq,
					IRQF_DISABLED, pdev->name, host);
		if (ret) {
			FUNC_PRINT("Unable to Card detect IRQ");
			goto err_irq2;
		}
	} else
		host->connect = 1;

	platform_set_drvdata(pdev, host);
	mmc_add_host(mmc);

	return 0;

err_irq2:
	free_irq(host->irq, host);
err_irq:
	dma_free_coherent(NULL, SDIO_ADMA_ALIGN_SIZE,
		(void *)host->adma_align_virt_addr, host->adma_align_addr);
err_align_buf:
	dma_free_coherent(NULL, SDIO_ADMA_BUF_SIZE,
		(void *)host->adma_virt_addr, host->adma_addr);
err:
	printk(KERN_ERR "%s: error!!(%d)\n", __func__, ret);
	if (host)
		mmc_free_host(mmc);
	return ret;

}

static int
emxx_sdio_remove(struct platform_device *pdev)
{
	struct emxx_mmc_host *host = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	if (host) {
		flush_scheduled_work();

		mmc_remove_host(host->mmc);

		dma_free_coherent(NULL, SDIO_ADMA_ALIGN_SIZE,
		  (void *)host->adma_align_virt_addr, host->adma_align_addr);

		dma_free_coherent(NULL, SDIO_ADMA_BUF_SIZE,
		  (void *)host->adma_virt_addr, host->adma_addr);

		emxx_reset_device(host->reset);
		emxx_close_clockgate(host->clk | host->sclk);

		if (host->detect_irq)
			free_irq(host->detect_irq, host);
		free_irq(host->irq, host);

		mmc_free_host(host->mmc);
	}
	return 0;
}

void emxx_sdio_idle_suspend(void)
{
	int i;

	for (i = 0; i < 3; i++) {
		if (g_host[i] != NULL && g_host[i]->connect == 1)
			g_host[i]->idle_suspend = 1;
	}
}

#ifdef CONFIG_PM
static int
emxx_sdio_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct emxx_mmc_host *host = platform_get_drvdata(pdev);
	int ret = 0;

	if (!host || host->suspended)
		return 0;

	/* detect interrupt disable */
	if (host->detect_irq)
		disable_irq(host->detect_irq);

	FUNC_PRINT("in\n");
	ret = mmc_suspend_host(host->mmc, state);
	if (ret == 0)
		host->suspended = 1;
	FUNC_PRINT("out\n");

	return ret;
}

/* Routine to resume the MMC device */
static int
emxx_sdio_resume(struct platform_device *pdev)
{
	struct emxx_mmc_host *host = platform_get_drvdata(pdev);
	int ret = 0;
	unsigned long flag;
	u32 val;

	if (!host || !host->suspended)
		return 0;

	if (host->detect_irq) {
		/* detect interrupt enable */
		enable_irq(host->detect_irq);

		local_irq_save(flag);
		val = gpio_get_value(host->detect_gpio);
		if (val) {
			host->connect = 0;
			set_irq_type(host->detect_irq, IRQ_TYPE_LEVEL_LOW);
		} else {
			host->connect = 1;
			set_irq_type(host->detect_irq, IRQ_TYPE_LEVEL_HIGH);
		}
		local_irq_restore(flag);
	}
	FUNC_PRINT("in\n");
	ret = mmc_resume_host(host->mmc);
	FUNC_PRINT("out\n");

	host->suspended = 0;

	return ret;
}

#else
#define emxx_sdio_suspend	NULL
#define emxx_sdio_resume	NULL
#endif

static struct platform_driver emxx_mmc_driver = {
	.probe		= emxx_sdio_probe,
	.remove		= emxx_sdio_remove,
	.suspend	= emxx_sdio_suspend,
	.resume		= emxx_sdio_resume,
	.driver		= {
		.name = DRIVER_NAME,
	},
};

struct platform_driver emxx_mmc_driver2 = {
	.probe		= emxx_sdio_probe,
	.remove		= emxx_sdio_remove,
	.suspend	= emxx_sdio_suspend,
	.resume		= emxx_sdio_resume,
	.driver		= {
		.name = DRIVER_NAME2,
	},
};
EXPORT_SYMBOL(emxx_mmc_driver2);

static int __init emxx_sdio_init(void)
{
	return platform_driver_register(&emxx_mmc_driver);
}

static void __exit emxx_sdio_cleanup(void)
{
	platform_driver_unregister(&emxx_mmc_driver);
}

module_init(emxx_sdio_init);
module_exit(emxx_sdio_cleanup);

MODULE_DESCRIPTION("EMXX SDIO High Speed Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_AUTHOR("Renesas Electronics Corporation");
