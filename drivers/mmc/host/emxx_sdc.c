/*
 *  File Name		: linux/drivers/mmc/host/emxx_sdc_sd.c
 *  Function		: MMC
 *  Release Version	: Ver 1.05
 *  Release Date	: 2010/10/18
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
#include <linux/mmc/sd.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/smu.h>
#include <mach/pmu.h>
#include <mach/dma.h>
#include <mach/pwc.h>
#include <mach/gpio.h>

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#include <mach/pm.h>
#endif

/* #define MMC_SPEED_NORMAL */

#include "emxx_sdc.h"

#define DRIVER_NAME	"emxx_sdc"


#define MMC_CLK_ON	emxx_open_clockgate(EMXX_CLK_SDC_H | EMXX_CLK_SDC)
#define MMC_CLK_OFF	emxx_close_clockgate(EMXX_CLK_SDC_H | EMXX_CLK_SDC)

/* #define DEBUG_PRINT */
#ifdef DEBUG_PRINT
#define FUNC_PRINT(FMT, ARGS...)	\
		printk(KERN_INFO "%s(): " FMT, __func__, ##ARGS)
#else
#define FUNC_PRINT(FMT, ARGS...)
#endif

static inline u32 emxx_sdc_info_read(void)
{
	u32 info;

	info = readl(EMXX_MMC_INFO1) & 0xffff;
	info |= ((readl(EMXX_MMC_INFO2) & 0xffff) << 16);

	return info;
}

static inline void emxx_sdc_mask_set(u32 mask)
{
	writel(mask & 0xffff, EMXX_MMC_INFO1_MASK);
	writel((mask >> 16) & 0xffff, EMXX_MMC_INFO2_MASK);
}

/*
 * Notify the core about command completion
 */
static void
emxx_sdc_data_done(struct emxx_sdc_host *host, struct mmc_command *cmd)
{
	struct mmc_data *data = host->data;
	u32 val;

	writel(MMC_STOP_STOP, EMXX_MMC_STOP);

	if (data != NULL) {
		dma_unmap_sg(mmc_dev(host->mmc), data->sg,
					host->dma_num, host->dma_dir);

		if (!data->error)
			data->bytes_xfered = data->blksz * data->blocks;
	}
	host->data_len = 0;

	writel(MMC_CC_EXT_MODE_FIFO, EMXX_MMC_CC_EXT_MODE);
	val = readl(EMXX_MMC_BUSIF_CTRL) & ~MMC_BUSIF_DMA_ENABLE;
	writel(val, EMXX_MMC_BUSIF_CTRL);

	host->data = NULL;
	host->cmd = NULL;
	host->req = NULL;
	host->stop = NULL;

	/* Clear interrupt */
	writel(MMC_DMAINT_ALL, EMXX_MMC_INT_CLR);
	val = readl(EMXX_MMC_INT_MASK) | MMC_DMAINT_ALL;
	writel(val, EMXX_MMC_INT_MASK);

	writel(0, EMXX_MMC_INFO1);
	writel(0, EMXX_MMC_INFO2);
	host->info_mask = MMC_INFO_MASK;
	emxx_sdc_mask_set(host->info_mask);

	MMC_CLK_OFF;

	mmc_request_done(host->mmc, cmd->mrq);
}

/*
 * Notify the core about command completion
 */
static void
emxx_sdc_cmd_done(struct emxx_sdc_host *host, struct mmc_command *cmd)
{
	u32 val;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			val = readl(EMXX_MMC_RSP0);
			cmd->resp[3] = val << 8;
			val = readl(EMXX_MMC_RSP1);
			cmd->resp[3] |= (val & 0xff) << 24;
			cmd->resp[2] = val >> 8;
			val = readl(EMXX_MMC_RSP2);
			cmd->resp[2] |= val << 8;
			val = readl(EMXX_MMC_RSP3);
			cmd->resp[2] |= (val & 0xff) << 24;
			cmd->resp[1] = val >> 8;
			val = readl(EMXX_MMC_RSP4);
			cmd->resp[1] |= val << 8;
			val = readl(EMXX_MMC_RSP5);
			cmd->resp[1] |= (val & 0xff) << 24;
			cmd->resp[0] = val >> 8;
			val = readl(EMXX_MMC_RSP6);
			cmd->resp[0] |= val << 8;
			val = readl(EMXX_MMC_RSP7);
			cmd->resp[0] |= (val & 0xff) << 24;
			FUNC_PRINT("%s: CMD=%d RSP %x %x %x %x\n",
				__func__, cmd->opcode, cmd->resp[0],
				cmd->resp[1], cmd->resp[2], cmd->resp[3]);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = readl(EMXX_MMC_RSP0);
			cmd->resp[0] |= (readl(EMXX_MMC_RSP1) << 16);
			FUNC_PRINT("%s: CMD=%d RSP %x\n", __func__,
						cmd->opcode, cmd->resp[0]);
		}
	}

	if (cmd->flags & MMC_RSP_BUSY) {
		/* wait DATA0 == 1 */
		while (!(emxx_sdc_info_read() & MMC_INFO_DAT0))
			;
	}

	if (host->app_mode)
		host->app_mode = 0;
	else if (cmd->opcode == 55)
		host->app_mode = 1;

	if (host->data == NULL) {
		/* Clear interrupt */
		writel(0, EMXX_MMC_INFO1);
		writel(0, EMXX_MMC_INFO2);
		MMC_CLK_OFF;
		host->req = NULL;
		host->cmd = NULL;
		mmc_request_done(host->mmc, cmd->mrq);
	} else if (host->data->error) {
		emxx_sdc_data_done(host, host->cmd);
	} else {
		host->info_mask |= MMC_INFO_EOC;
		emxx_sdc_mask_set(host->info_mask);
		if (host->data && host->data->flags & MMC_DATA_WRITE)
			writel(1, EMXX_MMC_TRANS_START);
	}
}

/*
 * Configure the Responce type
 */
static void
emxx_sdc_start_cmd(struct emxx_sdc_host *host,
			struct mmc_command *cmd, u16 cmddat)
{
	u32 val;

	cmddat |= cmd->opcode;

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_R1: /* short CRC, OPCODE */
		cmddat |= MMC_CMD_RSP_R1;
		break;
	case MMC_RSP_R1B:/* short CRC, OPCODE, BUSY */
		cmddat |= MMC_CMD_RSP_R1B;
		break;
	case MMC_RSP_R2: /* long 136 bit + CRC */
		cmddat |= MMC_CMD_RSP_R2;
		break;
	case MMC_RSP_R3: /* short */
		cmddat |= MMC_CMD_RSP_R3;
		break;
	}

	if (host->app_mode)
		cmddat |= MMC_CMD_ACMD;

	val = readl(EMXX_MMC_STOP) & ~MMC_STOP_STOP;
	writel(val, EMXX_MMC_STOP);

	writel(cmd->arg & 0xffff, EMXX_MMC_ARG0);
	writel(cmd->arg >> 16,    EMXX_MMC_ARG1);

	FUNC_PRINT("CMD=%d arg=0x%x flag=0x%x retry=%d reg=0x%x\n",
		cmd->opcode, cmd->arg, cmd->flags, cmd->retries, cmddat);

	/* Send command */
	writel(cmddat, EMXX_MMC_CMD);
}

static void emxx_sdc_start_dma(struct emxx_sdc_host *host)
{
	struct mmc_data *data = host->data;
	u32 len, addr_h, addr_l;

	FUNC_PRINT("%d %d\n", host->dma_count, host->dma_num);

	writel(MMC_RST_CTRL_RESET,   EMXX_MMC_RST_CTRL);
	writel(MMC_RST_CTRL_UNRESET, EMXX_MMC_RST_CTRL);

	if (host->dma_count >= host->dma_num) {
		host->info_mask &= ~MMC_INFO_RWEND;
		emxx_sdc_mask_set(host->info_mask);
		return;
	}

	len = host->dma_len[host->dma_count];
	if (len <= 512) {
		writel(len, EMXX_MMC_SECTOR_LEN0);
		writel(1, EMXX_MMC_BLOCK_LEN);
	} else {
		writel(data->blksz, EMXX_MMC_SECTOR_LEN0);
		writel(len / 512, EMXX_MMC_BLOCK_LEN);
	}

	addr_l = host->dma_address[host->dma_count] & 0xffff;
	addr_h = (host->dma_address[host->dma_count] >> 16) & 0xffff;

	writel(addr_l, EMXX_MMC_TXMEM_ADDR0L);
	writel(addr_l, EMXX_MMC_RXMEM_ADDR0L);
	writel(addr_h, EMXX_MMC_TXMEM_ADDR0H);
	writel(addr_h, EMXX_MMC_RXMEM_ADDR0H);

	if (data->flags & MMC_DATA_READ)
		writel(1, EMXX_MMC_TRANS_START);
	else {
		if (host->dma_count > 0)
			writel(1, EMXX_MMC_TRANS_START);
	}
	host->dma_count++;
}

/*
 * MMC controller IRQ handler
 */
static irqreturn_t
emxx_sdc_irq(int irq, void *dev_id)
{
	struct emxx_sdc_host *host = (struct emxx_sdc_host *)dev_id;
	u32 info, dma_int, end_trans = 0;

	info = emxx_sdc_info_read() & ~host->info_mask;
	dma_int = readl(EMXX_MMC_INT_ORG);

	if (host->cmd == NULL) {
		writel(0, EMXX_MMC_INFO1);
		writel(0, EMXX_MMC_INFO2);
		return IRQ_HANDLED;
	}

	if (info & MMC_INFO_ALLERR) {
		end_trans = 1;
		if (info & MMC_INFO_CTO) {
			FUNC_PRINT("CMD=%d CMD Timeout\n",
						host->cmd->opcode);
			host->cmd->error = -ETIMEDOUT;
		} else if (info & MMC_INFO_CRCERR) {
			FUNC_PRINT("CMD=%d CRC error\n",
						host->cmd->opcode);
			host->cmd->error = -EILSEQ;
		} else {
			FUNC_PRINT("CMD=%d Other error.(0x%x)\n",
						host->cmd->opcode, info);
			host->cmd->error = -EILSEQ;
		}
		if (host->data) {
			if (info & MMC_INFO_DTO) {
				FUNC_PRINT("CMD=%d DATA Timeout\n",
						host->cmd->opcode);
				host->data->error = -ETIMEDOUT;
			} else {
				FUNC_PRINT("CMD=%d Other error.(0x%x)\n",
						host->cmd->opcode, info);
				host->data->error = -EILSEQ;
			}
		}
	}

	if (dma_int & MMC_DMAERR) {
		FUNC_PRINT("CMD=%d DMA error.(0x%x 0x%x)\n",
			host->cmd->opcode, readl(EMXX_MMC_ERR_MHADDR_L),
			readl(EMXX_MMC_ERR_MHADDR_H));
		host->data->error = -EILSEQ;
		end_trans = 1;
	}

	if (dma_int & MMC_DMAINT) {
		writel(MMC_DMAINT, EMXX_MMC_INT_CLR);
		emxx_sdc_start_dma(host);
	}

	if ((info & MMC_INFO_EOC) || end_trans) {
		emxx_sdc_cmd_done(host, host->cmd);
		return IRQ_HANDLED;
	}

	if ((info & MMC_INFO_RWEND)) {
		emxx_sdc_data_done(host, host->cmd);
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

/*
 * IRQ for handling card insertion and removal
 */
static irqreturn_t
emxx_sdc_detect_irq(int irq, void *dev_id)
{
	struct emxx_sdc_host *host = (struct emxx_sdc_host *)dev_id;
	unsigned int tmp_data;

	FUNC_PRINT("\n");
	tmp_data = gpio_get_value(GPIO_SDC_CD);
	if (tmp_data)
		host->connect = 0;
	else
		host->connect = 1;

	mmc_detect_change(host->mmc, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

/*
 * Configure block leangth for MMC/SD cards and intiate the transfer.
 */
static void
emxx_sdc_setup_data(struct emxx_sdc_host *host, struct mmc_data *data)
{
	int i;
	u32 val;

	host->data_len = data->blksz * data->blocks;

	FUNC_PRINT("DATA(%s): size=%d num=%d\n",
		(data->flags & MMC_DATA_READ) ? "Read" : "Write",
		data->blksz, data->blocks);

	writel(data->blksz,  EMXX_MMC_SIZE);
	writel(data->blocks, EMXX_MMC_SECCNT);

	host->dma_dir = (data->flags & MMC_DATA_READ) ?
				DMA_FROM_DEVICE : DMA_TO_DEVICE;

	host->dma_num = dma_map_sg(mmc_dev(host->mmc), data->sg,
				data->sg_len, host->dma_dir);

	writel(MMC_CC_EXT_MODE_SD_DMA, EMXX_MMC_CC_EXT_MODE);

	host->dma_count = 0;

	for (i = 0; i < host->dma_num; i++) {
		host->dma_address[i] = sg_dma_address(&data->sg[i]);
		host->dma_len[i] = sg_dma_len(&data->sg[i]);
	}

	val = readl(EMXX_MMC_BUSIF_CTRL) | MMC_BUSIF_DMA_ENABLE;
	writel(val, EMXX_MMC_BUSIF_CTRL);
	writel(MMC_DMAINT_ALL, EMXX_MMC_INT_CLR);
	val = readl(EMXX_MMC_INT_MASK) & ~MMC_DMAINT_ALL;
	writel(val, EMXX_MMC_INT_MASK);

	emxx_sdc_start_dma(host);
}

/*
 * Request function. for read/write operation
 */
static void
emxx_sdc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct emxx_sdc_host *host = mmc_priv(mmc);
	u16 cmddat = 0;
	u32 info;

	if (host->connect == 0) {
		req->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, req);
		return;
	}

	host->req = req;
	host->stop = req->stop;
	host->cmd = req->cmd;

	MMC_CLK_ON;
	info = emxx_sdc_info_read();

	/* Clear stat */
	writel(0, EMXX_MMC_INFO1);
	writel(0, EMXX_MMC_INFO2);
	req->cmd->error = 0;

	if (req->data) {
		host->data = req->data;
		req->data->error = 0;

		emxx_sdc_setup_data(host, req->data);

		cmddat |= MMC_CMD_DATA;

		if (host->data->flags & MMC_DATA_READ)
			cmddat |= MMC_CMD_READ;

		if (host->data->blocks > 1) {
			cmddat |= MMC_CMD_MULTI;
			writel(MMC_STOP_MULTI, EMXX_MMC_STOP);
		}
	}

	emxx_sdc_start_cmd(host, req->cmd, cmddat);
}

static void emxx_sdc_power(struct emxx_sdc_host *host, u32 power_on)
{
	u32 val;

	if (power_on) {
		MMC_CLK_ON;
		emxx_unreset_device(EMXX_RST_SDC);

		writel(MMC_SOFT_RST_RESET,     EMXX_MMC_SOFT_RST);
		writel(MMC_SOFT_RST_UNRESET,   EMXX_MMC_SOFT_RST);
		emxx_sdc_mask_set(MMC_INFO_MASK);

		val = readl(EMXX_MMC_DMAMSK_CTRL) & ~MMC_DMAMSK_MASK;
		writel(val, EMXX_MMC_DMAMSK_CTRL);

		writel(MMC_DMAINT_ALL, EMXX_MMC_INT_CLR);
		writel(0, EMXX_MMC_INFO1);
		writel(0, EMXX_MMC_INFO2);
		writel(0, EMXX_SDIO_INFO1);
		writel(MMC_SDIO_INFO1_MASK, EMXX_SDIO_INFO1_MASK);
		writel(MMC_STOP_STOP, EMXX_MMC_STOP);
		writel(0x80EE, EMXX_MMC_OPTION);
		writel(MMC_BUSIF_MODE_SINGLE, EMXX_MMC_BUSIF_CTRL);
#ifdef CONFIG_MACH_EMGR
		writel(0x0100, EMXX_MMC_USER);
		writel(0x0000, EMXX_MMC_USER2);
#endif
		MMC_CLK_OFF;
	} else {
		host->clock = 0;
		host->bus_width = 0;
		MMC_CLK_ON;
		emxx_reset_device(EMXX_RST_SDC);
		MMC_CLK_OFF;
	}
}


/*
 * Configuring clock values.
 */
static void
emxx_sdc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct emxx_sdc_host *host = mmc_priv(mmc);
	u32 val, wDiv;

/*
	printk("clock=%d power=0x%x width=%d timing=%d\n",
		ios->clock, ios->power_mode, ios->bus_width, ios->timing);
*/

	if ((host->bus_width != ios->bus_width)
				&& (host->power_mode != MMC_POWER_OFF)) {
		MMC_CLK_ON;
		val = readl(EMXX_MMC_OPTION);
		if (ios->bus_width == MMC_BUS_WIDTH_4) {
			host->bus_width = MMC_BUS_WIDTH_4;
			val &= ~MMC_OPTION_WIDTH4;
		} else {
			host->bus_width = MMC_BUS_WIDTH_1;
			val |= MMC_OPTION_WIDTH4;
		}
		writel(val, EMXX_MMC_OPTION);
		MMC_CLK_OFF;
	}

	if ((ios->clock != host->clock)
			&& (host->power_mode != MMC_POWER_OFF)) {
		MMC_CLK_ON;
		/* Stop clock */
		writel(0, EMXX_MMC_CLK_CTRL);

		if (ios->clock >= host->base_clock / 2)
			wDiv = MMC_CLOCK_DIV2   | MMC_CLOCK_OFFEN;
		else if (ios->clock >= host->base_clock / 4)
			wDiv = MMC_CLOCK_DIV4   | MMC_CLOCK_OFFEN;
		else if (ios->clock >= host->base_clock / 8)
			wDiv = MMC_CLOCK_DIV8   | MMC_CLOCK_OFFEN;
		else if (ios->clock >= host->base_clock / 16)
			wDiv = MMC_CLOCK_DIV16  | MMC_CLOCK_OFFEN;
		else if (ios->clock >= host->base_clock / 32)
			wDiv = MMC_CLOCK_DIV32  | MMC_CLOCK_OFFEN;
		else if (ios->clock >= host->base_clock / 64)
			wDiv = MMC_CLOCK_DIV64  | MMC_CLOCK_OFFEN;
		else if (ios->clock >= host->base_clock / 128)
			wDiv = MMC_CLOCK_DIV128 | MMC_CLOCK_OFFEN;
		else if (ios->clock >= host->base_clock / 256)
			wDiv = MMC_CLOCK_DIV256;
		else
			wDiv = MMC_CLOCK_DIV512;

		if (ios->clock > 25000000)
			wDiv |= MMC_CLOCK_HIGH;

		wDiv |= MMC_CLOCK_EN;

		/* Start clock */
		writel(wDiv, EMXX_MMC_CLK_CTRL);
		MMC_CLK_OFF;
		host->clock = ios->clock;
	}

	if (host->power_mode != ios->power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
			emxx_sdc_power(host, 0);
			FUNC_PRINT("power off\n");
			break;
		case MMC_POWER_UP:
			emxx_sdc_power(host, 1);
			FUNC_PRINT("power up\n");
			break;
		case MMC_POWER_ON:
			FUNC_PRINT("power on\n");
			break;
		}
		host->power_mode = ios->power_mode;
	}
}

static int emxx_sdc_get_ro(struct mmc_host *mmc)
{
	return gpio_get_value(GPIO_SD_WP);
}

static struct mmc_host_ops emxx_sdc_ops = {
	.request	= emxx_sdc_request,
	.set_ios	= emxx_sdc_set_ios,
	.get_ro		= emxx_sdc_get_ro,
};

static int
emxx_sdc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct emxx_sdc_host *host = NULL;
	int ret = 0;
	unsigned int tmp_data;

	mmc = mmc_alloc_host(sizeof(struct emxx_sdc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto err;
	}

	mmc->ops   = &emxx_sdc_ops;
	mmc->f_min = 200000;
#ifdef MMC_SPEED_NORMAL
	mmc->f_max = 25000000;
#else
	mmc->f_max = 50000000;
#endif
	mmc->ocr_avail = MMC_VDD_28_29;
	mmc->caps = MMC_CAP_4_BIT_DATA;
	if (mmc->f_max > 25000000)
		mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

	/* MMC core transfer sizes tunable parameters */
	mmc->max_hw_segs   = EMXX_MAX_SEGS;
	mmc->max_phys_segs = EMXX_MAX_SEGS;
	mmc->max_seg_size  = EMXX_MAX_SEGS * 512;
	mmc->max_req_size  = EMXX_MAX_SEGS * 512;
	mmc->max_blk_size  = 512;
	mmc->max_blk_count = 65535;

	host      = mmc_priv(mmc);
	host->mmc = mmc;
	host->irq = INT_SDC_SYNC;
	host->detect_irq = gpio_to_irq(GPIO_SDC_CD);
	host->power_mode = MMC_POWER_OFF;
	host->info_mask = MMC_INFO_MASK;
	host->base_clock = MMC_BASE_CLOCK;
	host->bus_width = 10;	/* set dummy */
	host->clock = 0;

	mutex_init(&host->mutex);
	spin_lock_init(&host->lock);

	/* Request IRQ for MMC operations */
	ret = request_irq(host->irq, emxx_sdc_irq,
				IRQF_DISABLED, pdev->name, host);
	if (ret) {
		printk(KERN_ERR "Unable to IRQ");
		goto err;
	}

	set_irq_type(host->detect_irq, IRQ_TYPE_EDGE_BOTH);

	tmp_data = gpio_get_value(GPIO_SDC_CD);
	if (tmp_data)
		host->connect = 0;
	else
		host->connect = 1;

	/* Request IRQ for MMC card detect */
	ret = request_irq(host->detect_irq, emxx_sdc_detect_irq,
					IRQF_DISABLED, pdev->name, host);
	if (ret) {
		printk(KERN_ERR "Unable to Card detect IRQ");
		goto err_irq;
	}
	platform_set_drvdata(pdev, host);
	mmc_add_host(mmc);

	return 0;

err_irq:
	free_irq(host->irq, host);
err:
	printk(KERN_ERR "%s: error!!(%d)\n", __func__, ret);
	if (host)
		mmc_free_host(mmc);
	return ret;

}

static int
emxx_sdc_remove(struct platform_device *pdev)
{
	struct emxx_sdc_host *host = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	if (host) {
		flush_scheduled_work();

		mmc_remove_host(host->mmc);

		emxx_sdc_power(host, 0);

		free_irq(host->detect_irq, host);
		free_irq(host->irq, host);

		mmc_free_host(host->mmc);
	}

	return 0;
}

#ifdef CONFIG_PM
static int
emxx_sdc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct emxx_sdc_host *host = platform_get_drvdata(pdev);
	int ret = 0;

	if (!host || host->suspended)
		return 0;

	/* detect interrupt disable */
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
emxx_sdc_resume(struct platform_device *pdev)
{
	struct emxx_sdc_host *host = platform_get_drvdata(pdev);
	int ret = 0;

	if (!host || !host->suspended)
		return 0;

	/* detect interrupt enable */
	enable_irq(host->detect_irq);

	FUNC_PRINT("in\n");
	ret = mmc_resume_host(host->mmc);
	FUNC_PRINT("out\n");

	host->suspended = 0;

	return ret;
}
#else
#define emxx_sdc_suspend	NULL
#define emxx_sdc_resume	NULL
#endif

static struct platform_driver emxx_sdc_driver = {
	.probe		= emxx_sdc_probe,
	.remove		= emxx_sdc_remove,
	.suspend	= emxx_sdc_suspend,
	.resume		= emxx_sdc_resume,
	.driver		= {
			.name = DRIVER_NAME,
	},
};

static int __init emxx_sdc_init(void)
{
	return platform_driver_register(&emxx_sdc_driver);
}

static void __exit emxx_sdc_cleanup(void)
{
	platform_driver_unregister(&emxx_sdc_driver);
}

module_init(emxx_sdc_init);
module_exit(emxx_sdc_cleanup);

MODULE_DESCRIPTION("EMXX SDC High Speed Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_AUTHOR("Renesas Electronics Corporation");
