/*
 * Driver for sunxi SD/MMC host controllers
 * (C) Copyright 2014-2015 Reuuimlla Technology Co., Ltd.
 * (C) Copyright 2014-2015 Aaron Maoye <leafy.myeh@reuuimllatech.com>
 * (C) Copyright 2014-2015 O2S GmbH <www.o2s.ch>
 * (C) Copyright 2014-2015 David Lanzendörfer <david.lanzendoerfer@o2s.ch>
 * (C) Copyright 2014-2015 Hans de Goede <hdegoede@redhat.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <linux/clk.h>
#include <linux/clk-private.h>
#include <linux/clk/sunxi.h>

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/slot-gpio.h>

#include "sunxi-mmc.h"

static int sunxi_mmc_init_host(struct mmc_host *mmc)
{
	u32 rval;
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	int ret;

	ret =  clk_prepare_enable(smc_host->clk_ahb);
	if (ret) {
		dev_err(mmc_dev(smc_host->mmc), "AHB clk err %d\n", ret);
		return ret;
	}
	ret =  clk_prepare_enable(smc_host->clk_mod);
	if (ret) {
		dev_err(mmc_dev(smc_host->mmc), "MOD clk err %d\n", ret);
		clk_disable_unprepare(smc_host->clk_ahb);
		return ret;
	}

	/* reset controller */
	rval = mci_readl(smc_host, REG_GCTRL) | SDXC_HARDWARE_RESET;
	mci_writel(smc_host, REG_GCTRL, rval);

	mci_writel(smc_host, REG_FTRGL, 0x20070008);
	mci_writel(smc_host, REG_TMOUT, 0xffffffff);
	mci_writel(smc_host, REG_IMASK, smc_host->sdio_imask);
	mci_writel(smc_host, REG_RINTR, 0xffffffff);
	mci_writel(smc_host, REG_DBGC, 0xdeb);
	mci_writel(smc_host, REG_FUNS, 0xceaa0000);
	mci_writel(smc_host, REG_DLBA, smc_host->sg_dma);
	rval = mci_readl(smc_host, REG_GCTRL)|SDXC_INTERRUPT_ENABLE_BIT;
	rval &= ~SDXC_ACCESS_DONE_DIRECT;
	mci_writel(smc_host, REG_GCTRL, rval);

	return 0;
}

static void sunxi_mmc_exit_host(struct sunxi_mmc_host *smc_host)
{
	mci_writel(smc_host, REG_GCTRL, SDXC_HARDWARE_RESET);
	clk_disable_unprepare(smc_host->clk_ahb);
	clk_disable_unprepare(smc_host->clk_mod);
}

/* /\* UHS-I Operation Modes */
/*  * DS		25MHz	12.5MB/s	3.3V */
/*  * HS		50MHz	25MB/s		3.3V */
/*  * SDR12	25MHz	12.5MB/s	1.8V */
/*  * SDR25	50MHz	25MB/s		1.8V */
/*  * SDR50	100MHz	50MB/s		1.8V */
/*  * SDR104	208MHz	104MB/s		1.8V */
/*  * DDR50	50MHz	50MB/s		1.8V */
/*  * MMC Operation Modes */
/*  * DS		26MHz	26MB/s		3/1.8/1.2V */
/*  * HS		52MHz	52MB/s		3/1.8/1.2V */
/*  * HSDDR	52MHz	104MB/s		3/1.8/1.2V */
/*  * HS200	200MHz	200MB/s		1.8/1.2V */
/*  * */
/*  * Spec. Timing */
/*  * SD3.0 */
/*  * Fcclk    Tcclk   Fsclk   Tsclk   Tis     Tih     odly  RTis     RTih */
/*  * 400K     2.5us   24M     41ns    5ns     5ns     1     2209ns   41ns */
/*  * 25M      40ns    600M    1.67ns  5ns     5ns     3     14.99ns  5.01ns */
/*  * 50M      20ns    600M    1.67ns  6ns     2ns     3     14.99ns  5.01ns */
/*  * 50MDDR   20ns    600M    1.67ns  6ns     0.8ns   2     6.67ns   3.33ns */
/*  * 104M     9.6ns   600M    1.67ns  3ns     0.8ns   1     7.93ns   1.67ns */
/*  * 208M     4.8ns   600M    1.67ns  1.4ns   0.8ns   1     3.33ns   1.67ns */

/*  * 25M      40ns    300M    3.33ns  5ns     5ns     2     13.34ns   6.66ns */
/*  * 50M      20ns    300M    3.33ns  6ns     2ns     2     13.34ns   6.66ns */
/*  * 50MDDR   20ns    300M    3.33ns  6ns     0.8ns   1     6.67ns    3.33ns */
/*  * 104M     9.6ns   300M    3.33ns  3ns     0.8ns   0     7.93ns    1.67ns */
/*  * 208M     4.8ns   300M    3.33ns  1.4ns   0.8ns   0     3.13ns    1.67ns */

/*  * eMMC4.5 */
/*  * 400K     2.5us   24M     41ns    3ns     3ns     1     2209ns    41ns */
/*  * 25M      40ns    600M    1.67ns  3ns     3ns     3     14.99ns   5.01ns */
/*  * 50M      20ns    600M    1.67ns  3ns     3ns     3     14.99ns   5.01ns */
/*  * 50MDDR   20ns    600M    1.67ns  2.5ns   2.5ns   2     6.67ns    3.33ns */
/*  * 200M     5ns     600M    1.67ns  1.4ns   0.8ns   1     3.33ns    1.67ns */
/*  *\/ */

static void sunxi_mmc_init_idma_des(struct sunxi_mmc_host *host,
				    struct mmc_data *data)
{
	struct sunxi_idma_des *pdes = (struct sunxi_idma_des *)host->sg_cpu;
	struct sunxi_idma_des *pdes_pa = (struct sunxi_idma_des *)host->sg_dma;
	int i, max_len = (1 << host->idma_des_size_bits);

	for (i = 0; i < data->sg_len; i++) {
		pdes[i].config = SDXC_IDMAC_DES0_CH | SDXC_IDMAC_DES0_OWN |
				 SDXC_IDMAC_DES0_DIC;

		if (data->sg[i].length == max_len)
			pdes[i].buf_size = 0; /* 0 == max_len */
		else
			pdes[i].buf_size = data->sg[i].length;

		pdes[i].buf_addr_ptr1 = sg_dma_address(&data->sg[i]);
		pdes[i].buf_addr_ptr2 = (u32)&pdes_pa[i + 1];
	}

	pdes[0].config |= SDXC_IDMAC_DES0_FD;
	pdes[i - 1].config = SDXC_IDMAC_DES0_OWN | SDXC_IDMAC_DES0_LD;

	wmb(); /* Ensure idma_des hit main mem before we start the idmac */
}

static enum dma_data_direction sunxi_mmc_get_dma_dir(struct mmc_data *data)
{
	if (data->flags & MMC_DATA_WRITE)
		return DMA_TO_DEVICE;
	else
		return DMA_FROM_DEVICE;
}

static int sunxi_mmc_prepare_dma(struct sunxi_mmc_host *smc_host,
				 struct mmc_data *data)
{
	u32 dma_len;
	u32 i;
	u32 temp;
	struct scatterlist *sg;

	dma_len = dma_map_sg(mmc_dev(smc_host->mmc), data->sg, data->sg_len,
			     sunxi_mmc_get_dma_dir(data));
	if (dma_len == 0) {
		dev_err(mmc_dev(smc_host->mmc), "dma_map_sg failed\n");
		return -ENOMEM;
	}

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3 || sg->length & 3) {
			dev_err(mmc_dev(smc_host->mmc),
				"unaligned scatterlist: os %x length %d\n",
				sg->offset, sg->length);
			return -EINVAL;
		}
	}

	sunxi_mmc_init_idma_des(smc_host, data);

	temp = mci_readl(smc_host, REG_GCTRL);
	temp |= SDXC_DMA_ENABLE_BIT;
	mci_writel(smc_host, REG_GCTRL, temp);
	temp |= SDXC_DMA_RESET;
	mci_writel(smc_host, REG_GCTRL, temp);
	mci_writel(smc_host, REG_DMAC, SDXC_IDMAC_SOFT_RESET);

	if (!(data->flags & MMC_DATA_WRITE))
		mci_writel(smc_host, REG_IDIE, SDXC_IDMAC_RECEIVE_INTERRUPT);

	mci_writel(smc_host, REG_DMAC, SDXC_IDMAC_FIX_BURST | SDXC_IDMAC_IDMA_ON);

	return 0;
}

static void sunxi_mmc_send_manual_stop(struct sunxi_mmc_host *host,
				       struct mmc_request *req)
{
	u32 cmd_val = SDXC_START | SDXC_RESPONSE_EXPIRE | SDXC_STOP_ABORT_CMD
			| SDXC_CHECK_RESPONSE_CRC | MMC_STOP_TRANSMISSION;
	u32 ri = 0;
	unsigned long expire = jiffies + msecs_to_jiffies(1000);

	mci_writel(host, REG_CARG, 0);
	mci_writel(host, REG_CMDR, cmd_val);

	do {
		ri = mci_readl(host, REG_RINTR);
	} while (!(ri & (SDXC_COMMAND_DONE | SDXC_INTERRUPT_ERROR_BIT)) &&
		 time_before(jiffies, expire));

	if (ri & SDXC_INTERRUPT_ERROR_BIT) {
		dev_err(mmc_dev(host->mmc), "send stop command failed\n");
		if (req->stop)
			req->stop->resp[0] = -ETIMEDOUT;
	} else {
		if (req->stop)
			req->stop->resp[0] = mci_readl(host, REG_RESP0);
	}

	mci_writel(host, REG_RINTR, 0xffff);
}

static void sunxi_mmc_dump_errinfo(struct sunxi_mmc_host *smc_host)
{
	struct mmc_command *cmd = smc_host->mrq->cmd;
	struct mmc_data *data = smc_host->mrq->data;

	/* For some cmds timeout is normal with sd/mmc cards */
	if ((smc_host->int_sum & SDXC_INTERRUPT_ERROR_BIT) == SDXC_RESPONSE_TIMEOUT &&
			(cmd->opcode == SD_IO_SEND_OP_COND || cmd->opcode == SD_IO_RW_DIRECT))
		return;

	dev_err(mmc_dev(smc_host->mmc),
		"smc %d err, cmd %d,%s%s%s%s%s%s%s%s%s%s !!\n",
		smc_host->mmc->index, cmd->opcode,
		data ? (data->flags & MMC_DATA_WRITE ? " WR" : " RD") : "",
		smc_host->int_sum & SDXC_RESPONSE_ERROR     ? " RE"     : "",
		smc_host->int_sum & SDXC_RESPONSE_CRC_ERROR  ? " RCE"    : "",
		smc_host->int_sum & SDXC_DATA_CRC_ERROR  ? " DCE"    : "",
		smc_host->int_sum & SDXC_RESPONSE_TIMEOUT ? " RTO"    : "",
		smc_host->int_sum & SDXC_DATA_TIMEOUT ? " DTO"    : "",
		smc_host->int_sum & SDXC_FIFO_RUN_ERROR  ? " FE"     : "",
		smc_host->int_sum & SDXC_HARD_WARE_LOCKED ? " HL"     : "",
		smc_host->int_sum & SDXC_START_BIT_ERROR ? " SBE"    : "",
		smc_host->int_sum & SDXC_END_BIT_ERROR   ? " EBE"    : ""
		);
}

static void sunxi_mmc_finalize_request(struct sunxi_mmc_host *host)
{
	struct mmc_request *mrq;
	unsigned long iflags;

	spin_lock_irqsave(&host->lock, iflags);

	mrq = host->mrq;
	if (!mrq) {
		spin_unlock_irqrestore(&host->lock, iflags);
		dev_err(mmc_dev(host->mmc), "no request to finalize\n");
		return;
	}

	if (host->int_sum & SDXC_INTERRUPT_ERROR_BIT) {
		sunxi_mmc_dump_errinfo(host);
		mrq->cmd->error = -ETIMEDOUT;
		if (mrq->data)
			mrq->data->error = -ETIMEDOUT;
		if (mrq->stop)
			mrq->stop->error = -ETIMEDOUT;
	} else {
		if (mrq->cmd->flags & MMC_RSP_136) {
			mrq->cmd->resp[0] = mci_readl(host, REG_RESP3);
			mrq->cmd->resp[1] = mci_readl(host, REG_RESP2);
			mrq->cmd->resp[2] = mci_readl(host, REG_RESP1);
			mrq->cmd->resp[3] = mci_readl(host, REG_RESP0);
		} else {
			mrq->cmd->resp[0] = mci_readl(host, REG_RESP0);
		}
		if (mrq->data)
			mrq->data->bytes_xfered =
				mrq->data->blocks * mrq->data->blksz;
	}

	if (mrq->data) {
		struct mmc_data *data = mrq->data;
		u32 temp;

		mci_writel(host, REG_IDST, 0x337);
		mci_writel(host, REG_DMAC, 0);
		temp = mci_readl(host, REG_GCTRL);
		mci_writel(host, REG_GCTRL, temp|SDXC_DMA_RESET);
		temp &= ~SDXC_DMA_ENABLE_BIT;
		mci_writel(host, REG_GCTRL, temp);
		temp |= SDXC_FIFO_RESET;
		mci_writel(host, REG_GCTRL, temp);
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				     sunxi_mmc_get_dma_dir(data));
	}

	mci_writel(host, REG_RINTR, 0xffff);

	dev_dbg(mmc_dev(host->mmc), "req done, resp %08x %08x %08x %08x\n",
		mrq->cmd->resp[0], mrq->cmd->resp[1],
		mrq->cmd->resp[2], mrq->cmd->resp[3]);

	host->mrq = NULL;
	host->int_sum = 0;
	host->wait_dma = 0;

	spin_unlock_irqrestore(&host->lock, iflags);

	if (mrq->data && mrq->data->error) {
		dev_err(mmc_dev(host->mmc),
			"data error, sending stop command\n");
		sunxi_mmc_send_manual_stop(host, mrq);
	}

	mmc_request_done(host->mmc, mrq);
}

static irqreturn_t sunxi_mmc_irq(int irq, void *dev_id)
{
	struct sunxi_mmc_host *host = dev_id;
	u32 finalize = 0;
	u32 sdio_int = 0;
	u32 msk_int;
	u32 idma_int;

	spin_lock(&host->lock);

	idma_int  = mci_readl(host, REG_IDST);
	msk_int   = mci_readl(host, REG_MISTA);

	dev_dbg(mmc_dev(host->mmc), "irq: rq %p mi %08x idi %08x\n",
		host->mrq, msk_int, idma_int);

	if (host->mrq) {
		if (idma_int & SDXC_IDMAC_RECEIVE_INTERRUPT)
			host->wait_dma = 0;

		host->int_sum |= msk_int;

		/* Wait for COMMAND_DONE on RESPONSE_TIMEOUT before finishing the req */
		if ((host->int_sum & SDXC_RESPONSE_TIMEOUT) &&
				!(host->int_sum & SDXC_COMMAND_DONE))
			mci_writel(host, REG_IMASK,
				   host->sdio_imask | SDXC_COMMAND_DONE);
		else if (host->int_sum & SDXC_INTERRUPT_ERROR_BIT)
			finalize = 1; /* Don't wait for dma on error */
		else if (host->int_sum & SDXC_INTERRUPT_DONE_BIT && !host->wait_dma)
			finalize = 1; /* Done */

		if (finalize) {
			mci_writel(host, REG_IMASK, host->sdio_imask);
			mci_writel(host, REG_IDIE, 0);
		}
	}

	if (msk_int & SDXC_SDIO_INTERRUPT)
		sdio_int = 1;

	mci_writel(host, REG_RINTR, msk_int);
	mci_writel(host, REG_IDST, idma_int);

	spin_unlock(&host->lock);

	if (finalize)
		tasklet_schedule(&host->tasklet);

	if (sdio_int)
		mmc_signal_sdio_irq(host->mmc);

	return IRQ_HANDLED;
}

static void sunxi_mmc_tasklet(unsigned long data)
{
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *) data;
	sunxi_mmc_finalize_request(smc_host);
}

static void sunxi_mmc_oclk_onoff(struct sunxi_mmc_host *host, u32 oclk_en)
{
	unsigned long expire = jiffies + msecs_to_jiffies(2000);
	u32 rval;

	rval = mci_readl(host, REG_CLKCR);
	rval &= ~(SDXC_CARD_CLOCK_ON | SDXC_LOW_POWER_ON);

	if (oclk_en)
		rval |= SDXC_CARD_CLOCK_ON;

	mci_writel(host, REG_CLKCR, rval);

	rval = SDXC_START | SDXC_UPCLK_ONLY | SDXC_WAIT_PRE_OVER;
	if (host->voltage_switching)
		rval |= SDXC_VOLTAGE_SWITCH;
	mci_writel(host, REG_CMDR, rval);

	do {
		rval = mci_readl(host, REG_CMDR);
	} while (time_before(jiffies, expire) && (rval & SDXC_START));

	if (rval & SDXC_START) {
		dev_err(mmc_dev(host->mmc), "fatal err update clk timeout\n");
		host->ferror = 1;
	}
}

struct sunxi_mmc_clk_dly mmc_clk_dly[MMC_CLK_MOD_NUM] = {
	{ MMC_CLK_400K, 0, 7 },
	{ MMC_CLK_25M, 0, 5 },
	{ MMC_CLK_50M, 3, 5 },
	{ MMC_CLK_50MDDR, 2, 4 },
	{ MMC_CLK_50MDDR_8BIT, 2, 4 },
	{ MMC_CLK_100M, 1, 4 },
	{ MMC_CLK_200M, 1, 4 },
};

static void sunxi_mmc_clk_set_rate(struct sunxi_mmc_host *smc_host,
				   unsigned int rate)
{
	u32 newrate;
	u32 src_clk;
	u32 oclk_dly;
	u32 sclk_dly;
	u32 temp;
	struct sunxi_mmc_clk_dly *dly = NULL;
	struct clk_hw *hw = __clk_get_hw(smc_host->clk_mod);

	newrate = clk_round_rate(smc_host->clk_mod, rate);
	if (smc_host->clk_mod_rate == newrate) {
		dev_dbg(mmc_dev(smc_host->mmc), "clk already %d, rounded %d\n",
			rate, newrate);
		return;
	}

	dev_dbg(mmc_dev(smc_host->mmc), "setting clk to %d, rounded %d\n",
		rate, newrate);

	/* setting clock rate */
	clk_disable(smc_host->clk_mod);
	clk_set_rate(smc_host->clk_mod, newrate);
	clk_enable(smc_host->clk_mod);
	smc_host->clk_mod_rate = newrate = clk_get_rate(smc_host->clk_mod);
	dev_dbg(mmc_dev(smc_host->mmc), "clk is now %d\n", newrate);

	sunxi_mmc_oclk_onoff(smc_host, 0);
	/* clear internal divider */
	temp = mci_readl(smc_host, REG_CLKCR);
	temp &= ~0xff;
	mci_writel(smc_host, REG_CLKCR, temp);

	/* determine delays */
	if (rate <= 400000) {
		dly = &mmc_clk_dly[MMC_CLK_400K];
	} else if (rate <= 25000000) {
		dly = &mmc_clk_dly[MMC_CLK_25M];
	} else if (rate <= 50000000) {
		if (smc_host->ddr) {
			if (smc_host->bus_width == 8)
				dly = &mmc_clk_dly[MMC_CLK_50MDDR_8BIT];
			else
				dly = &mmc_clk_dly[MMC_CLK_50MDDR];
		} else {
			dly = &mmc_clk_dly[MMC_CLK_50M];
		}
	} else if (rate <= 104000000) {
		dly = &mmc_clk_dly[MMC_CLK_100M];
	} else if (rate <= 208000000) {
		dly = &mmc_clk_dly[MMC_CLK_200M];
	} else {
		dly = &mmc_clk_dly[MMC_CLK_50M];
	}

	oclk_dly = dly->oclk_dly;
	sclk_dly = dly->sclk_dly;

	src_clk = clk_get_rate(clk_get_parent(smc_host->clk_mod));

	if (src_clk >= 300000000 && src_clk <= 400000000) {
		if (oclk_dly)
			oclk_dly--;
		if (sclk_dly)
			sclk_dly--;
	}

	clk_sunxi_mmc_phase_control(hw, sclk_dly, oclk_dly);
	sunxi_mmc_oclk_onoff(smc_host, 1);

	/* oclk_onoff sets various irq status bits, clear these */
	mci_writel(smc_host, REG_RINTR,
		   mci_readl(smc_host, REG_RINTR) & ~SDXC_SDIO_INTERRUPT);
}

static void sunxi_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sunxi_mmc_host *host = mmc_priv(mmc);
	u32 temp;
	s32 err;

	/* Set the power state */
	switch (ios->power_mode) {
	case MMC_POWER_ON:
		break;

	case MMC_POWER_UP:
		if (!IS_ERR(host->vmmc)) {
			mmc_regulator_set_ocr(host->mmc, host->vmmc, ios->vdd);
			udelay(200);
		}

		err = sunxi_mmc_init_host(mmc);
		if (err) {
			host->ferror = 1;
			return;
		}
		enable_irq(host->irq);

		dev_dbg(mmc_dev(host->mmc), "power on!\n");
		host->ferror = 0;
		break;

	case MMC_POWER_OFF:
		dev_dbg(mmc_dev(host->mmc), "power off!\n");
		disable_irq(host->irq);
		sunxi_mmc_exit_host(host);
		if (!IS_ERR(host->vmmc))
			mmc_regulator_set_ocr(host->mmc, host->vmmc, 0);
		host->ferror = 0;
		break;
	}

	/* set bus width */
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		mci_writel(host, REG_WIDTH, SDXC_WIDTH1);
		host->bus_width = 1;
		break;
	case MMC_BUS_WIDTH_4:
		mci_writel(host, REG_WIDTH, SDXC_WIDTH4);
		host->bus_width = 4;
		break;
	case MMC_BUS_WIDTH_8:
		mci_writel(host, REG_WIDTH, SDXC_WIDTH8);
		host->bus_width = 8;
		break;
	}

	/* set ddr mode */
	temp = mci_readl(host, REG_GCTRL);
	if (ios->timing == MMC_TIMING_UHS_DDR50) {
		temp |= SDXC_DDR_MODE;
		host->ddr = 1;
	} else {
		temp &= ~SDXC_DDR_MODE;
		host->ddr = 0;
	}
	mci_writel(host, REG_GCTRL, temp);

	/* set up clock */
	if (ios->clock && ios->power_mode) {
		dev_dbg(mmc_dev(host->mmc), "ios->clock: %d\n", ios->clock);
		sunxi_mmc_clk_set_rate(host, ios->clock);
		usleep_range(50000, 55000);
	}
}

static void sunxi_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	unsigned long flags;
	u32 imask;

	spin_lock_irqsave(&smc_host->lock, flags);
	imask = mci_readl(smc_host, REG_IMASK);
	if (enable) {
		smc_host->sdio_imask = SDXC_SDIO_INTERRUPT;
		imask |= SDXC_SDIO_INTERRUPT;
	} else {
		smc_host->sdio_imask = 0;
		imask &= ~SDXC_SDIO_INTERRUPT;
	}
	mci_writel(smc_host, REG_IMASK, imask);
	spin_unlock_irqrestore(&smc_host->lock, flags);
}

static void sunxi_mmc_hw_reset(struct mmc_host *mmc)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	mci_writel(smc_host, REG_HWRST, 0);
	udelay(10);
	mci_writel(smc_host, REG_HWRST, 1);
	udelay(300);
}

static void sunxi_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sunxi_mmc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	unsigned long iflags;
	u32 imask = SDXC_INTERRUPT_ERROR_BIT;
	u32 cmd_val = SDXC_START | (cmd->opcode & 0x3f);
	u32 byte_cnt = 0;
	int ret;

	if (!mmc_gpio_get_cd(mmc) || host->ferror) {
		dev_dbg(mmc_dev(host->mmc), "no medium present\n");
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	if (data) {
		byte_cnt = data->blksz * data->blocks;
		mci_writel(host, REG_BLKSZ, data->blksz);
		mci_writel(host, REG_BCNTR, byte_cnt);
		ret = sunxi_mmc_prepare_dma(host, data);
		if (ret < 0) {
			dev_err(mmc_dev(host->mmc), "prepare DMA failed\n");
			cmd->error = ret;
			cmd->data->error = ret;
			mmc_request_done(host->mmc, mrq);
			return;
		}
	}

	if (cmd->opcode == MMC_GO_IDLE_STATE) {
		cmd_val |= SDXC_SEND_INIT_SEQUENCE;
		imask |= SDXC_COMMAND_DONE;
	}

	if (cmd->opcode == SD_SWITCH_VOLTAGE) {
		cmd_val |= SDXC_VOLTAGE_SWITCH;
		imask |= SDXC_VOLTAGE_CHANGE_DONE;
		host->voltage_switching = 1;
		sunxi_mmc_oclk_onoff(host, 1);
	}

	if (cmd->flags & MMC_RSP_PRESENT) {
		cmd_val |= SDXC_RESPONSE_EXPIRE;
		if (cmd->flags & MMC_RSP_136)
			cmd_val |= SDXC_LONG_RESPONSE;
		if (cmd->flags & MMC_RSP_CRC)
			cmd_val |= SDXC_CHECK_RESPONSE_CRC;

		if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
			cmd_val |= SDXC_DATA_EXPIRE | SDXC_WAIT_PRE_OVER;
			if (cmd->data->flags & MMC_DATA_STREAM) {
				imask |= SDXC_AUTO_COMMAND_DONE;
				cmd_val |= SDXC_SEQUENCE_MODE | SDXC_SEND_AUTO_STOP;
			}
			if (cmd->data->stop) {
				imask |= SDXC_AUTO_COMMAND_DONE;
				cmd_val |= SDXC_SEND_AUTO_STOP;
			} else
				imask |= SDXC_DATA_OVER;

			if (cmd->data->flags & MMC_DATA_WRITE)
				cmd_val |= SDXC_WRITE;
			else
				host->wait_dma = 1;
		} else
			imask |= SDXC_COMMAND_DONE;
	} else
		imask |= SDXC_COMMAND_DONE;

	dev_dbg(mmc_dev(host->mmc), "cmd %d(%08x) arg %x ie 0x%08x len %d\n",
		cmd_val & 0x3f, cmd_val, cmd->arg, imask,
		mrq->data ? mrq->data->blksz * mrq->data->blocks : 0);

	spin_lock_irqsave(&host->lock, iflags);
	host->mrq = mrq;
	mci_writel(host, REG_IMASK, host->sdio_imask | imask);
	spin_unlock_irqrestore(&host->lock, iflags);

	mci_writel(host, REG_CARG, cmd->arg);
	mci_writel(host, REG_CMDR, cmd_val);
}

static const struct of_device_id sunxi_mmc_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-mmc", },
	{ .compatible = "allwinner,sun5i-a13-mmc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_mmc_of_match);

static struct mmc_host_ops sunxi_mmc_ops = {
	.request	 = sunxi_mmc_request,
	.set_ios	 = sunxi_mmc_set_ios,
	.get_ro		 = mmc_gpio_get_ro,
	.get_cd		 = mmc_gpio_get_cd,
	.enable_sdio_irq = sunxi_mmc_enable_sdio_irq,
	.hw_reset	 = sunxi_mmc_hw_reset,
};

static int sunxi_mmc_resource_request(struct sunxi_mmc_host *host,
				      struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (of_device_is_compatible(np, "allwinner,sun4i-a10-mmc"))
		host->idma_des_size_bits = 13;
	else
		host->idma_des_size_bits = 16;

	host->vmmc = devm_regulator_get_optional(&pdev->dev, "vmmc");
	if (IS_ERR(host->vmmc) && PTR_ERR(host->vmmc) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	host->reg_base = devm_ioremap_resource(&pdev->dev,
			      platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(host->reg_base))
		return PTR_ERR(host->reg_base);

	host->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(host->clk_ahb)) {
		dev_err(&pdev->dev, "Could not get ahb clock\n");
		return PTR_ERR(host->clk_ahb);
	}

	host->clk_mod = devm_clk_get(&pdev->dev, "mod");
	if (IS_ERR(host->clk_mod)) {
		dev_err(&pdev->dev, "Could not get mod clock\n");
		return PTR_ERR(host->clk_mod);
	}

	/* Make sure the controller is in a sane state before enabling irqs */
	ret = sunxi_mmc_init_host(host->mmc);
	if (ret)
		return ret;

	host->irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, host->irq, sunxi_mmc_irq, 0,
			       "sunxi-mmc", host);
	if (ret == 0)
		disable_irq(host->irq);

	/* And put it back in reset */
	sunxi_mmc_exit_host(host);

	return ret;
}

static int sunxi_mmc_probe(struct platform_device *pdev)
{
	struct sunxi_mmc_host *host;
	struct mmc_host *mmc;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct sunxi_mmc_host), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "mmc alloc host failed\n");
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	spin_lock_init(&host->lock);
	tasklet_init(&host->tasklet, sunxi_mmc_tasklet, (unsigned long)host);

	ret = sunxi_mmc_resource_request(host, pdev);
	if (ret)
		goto error_free_host;

	host->sg_cpu = dma_alloc_coherent(&pdev->dev, PAGE_SIZE,
					  &host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		dev_err(&pdev->dev, "Failed to allocate DMA descriptor mem\n");
		ret = -ENOMEM;
		goto error_free_host;
	}

	mmc->ops		= &sunxi_mmc_ops;
	mmc->max_blk_count	= 8192;
	mmc->max_blk_size	= 4096;
	mmc->max_segs		= PAGE_SIZE / sizeof(struct sunxi_idma_des);
	mmc->max_seg_size	= (1 << host->idma_des_size_bits);
	mmc->max_req_size	= mmc->max_seg_size * mmc->max_segs;
	/* 400kHz ~ 50MHz */
	mmc->f_min		=   400000;
	mmc->f_max		= 50000000;
	/* available voltages */
	if (!IS_ERR(host->vmmc))
		mmc->ocr_avail = mmc_regulator_get_ocrmask(host->vmmc);
	else
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
		     MMC_CAP_SDIO_IRQ;
	mmc->caps2 |= MMC_CAP2_NO_PRESCAN_POWERUP;

	ret = mmc_of_parse(mmc);
	if (ret)
		goto error_free_dma;

	ret = mmc_add_host(mmc);
	if (ret)
		goto error_free_dma;

	dev_info(&pdev->dev, "base:0x%p irq:%u\n", host->reg_base, host->irq);
	platform_set_drvdata(pdev, mmc);
	return 0;

error_free_dma:
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
error_free_host:
	mmc_free_host(mmc);
	return ret;
}

static int sunxi_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct sunxi_mmc_host *host = mmc_priv(mmc);

	mmc_remove_host(mmc);
	sunxi_mmc_exit_host(host);
	tasklet_disable(&host->tasklet);
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
	mmc_free_host(mmc);

	return 0;
}

static struct platform_driver sunxi_mmc_driver = {
	.driver = {
		.name	= "sunxi-mmc",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(sunxi_mmc_of_match),
	},
	.probe		= sunxi_mmc_probe,
	.remove		= sunxi_mmc_remove,
};
module_platform_driver(sunxi_mmc_driver);

MODULE_DESCRIPTION("Allwinner's SD/MMC Card Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Lanzendörfer <david.lanzendoerfer@o2s.ch>");
MODULE_ALIAS("platform:sunxi-mmc");
