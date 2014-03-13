/*
 * Driver for sunxi SD/MMC host controllers
 * (C) Copyright 2007-2011 Reuuimlla Technology Co., Ltd.
 * (C) Copyright 2007-2011 Aaron Maoye <leafy.myeh@reuuimllatech.com>
 * (C) Copyright 2013-2014 O2S GmbH <www.o2s.ch>
 * (C) Copyright 2013-2014 David Lanzendörfer <david.lanzendoerfer@o2s.ch>
 * (C) Copyright 2013-2014 Hans de Goede <hdegoede@redhat.com>
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
#include <linux/reset.h>

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

/* register offset definitions */
#define SDXC_REG_GCTRL	(0x00) /* SMC Global Control Register */
#define SDXC_REG_CLKCR	(0x04) /* SMC Clock Control Register */
#define SDXC_REG_TMOUT	(0x08) /* SMC Time Out Register */
#define SDXC_REG_WIDTH	(0x0C) /* SMC Bus Width Register */
#define SDXC_REG_BLKSZ	(0x10) /* SMC Block Size Register */
#define SDXC_REG_BCNTR	(0x14) /* SMC Byte Count Register */
#define SDXC_REG_CMDR	(0x18) /* SMC Command Register */
#define SDXC_REG_CARG	(0x1C) /* SMC Argument Register */
#define SDXC_REG_RESP0	(0x20) /* SMC Response Register 0 */
#define SDXC_REG_RESP1	(0x24) /* SMC Response Register 1 */
#define SDXC_REG_RESP2	(0x28) /* SMC Response Register 2 */
#define SDXC_REG_RESP3	(0x2C) /* SMC Response Register 3 */
#define SDXC_REG_IMASK	(0x30) /* SMC Interrupt Mask Register */
#define SDXC_REG_MISTA	(0x34) /* SMC Masked Interrupt Status Register */
#define SDXC_REG_RINTR	(0x38) /* SMC Raw Interrupt Status Register */
#define SDXC_REG_STAS	(0x3C) /* SMC Status Register */
#define SDXC_REG_FTRGL	(0x40) /* SMC FIFO Threshold Watermark Registe */
#define SDXC_REG_FUNS	(0x44) /* SMC Function Select Register */
#define SDXC_REG_CBCR	(0x48) /* SMC CIU Byte Count Register */
#define SDXC_REG_BBCR	(0x4C) /* SMC BIU Byte Count Register */
#define SDXC_REG_DBGC	(0x50) /* SMC Debug Enable Register */
#define SDXC_REG_HWRST	(0x78) /* SMC Card Hardware Reset for Register */
#define SDXC_REG_DMAC	(0x80) /* SMC IDMAC Control Register */
#define SDXC_REG_DLBA	(0x84) /* SMC IDMAC Descriptor List Base Addre */
#define SDXC_REG_IDST	(0x88) /* SMC IDMAC Status Register */
#define SDXC_REG_IDIE	(0x8C) /* SMC IDMAC Interrupt Enable Register */
#define SDXC_REG_CHDA	(0x90)
#define SDXC_REG_CBDA	(0x94)

#define mci_readl(host, reg) \
	readl((host)->reg_base + SDXC_##reg)
#define mci_writel(host, reg, value) \
	writel((value), (host)->reg_base + SDXC_##reg)

/* global control register bits */
#define SDXC_SOFT_RESET			BIT(0)
#define SDXC_FIFO_RESET			BIT(1)
#define SDXC_DMA_RESET			BIT(2)
#define SDXC_INTERRUPT_ENABLE_BIT	BIT(4)
#define SDXC_DMA_ENABLE_BIT		BIT(5)
#define SDXC_DEBOUNCE_ENABLE_BIT	BIT(8)
#define SDXC_POSEDGE_LATCH_DATA		BIT(9)
#define SDXC_DDR_MODE			BIT(10)
#define SDXC_MEMORY_ACCESS_DONE		BIT(29)
#define SDXC_ACCESS_DONE_DIRECT		BIT(30)
#define SDXC_ACCESS_BY_AHB		BIT(31)
#define SDXC_ACCESS_BY_DMA		(0 << 31)
#define SDXC_HARDWARE_RESET \
	(SDXC_SOFT_RESET | SDXC_FIFO_RESET | SDXC_DMA_RESET)

/* clock control bits */
#define SDXC_CARD_CLOCK_ON		BIT(16)
#define SDXC_LOW_POWER_ON		BIT(17)

/* bus width */
#define SDXC_WIDTH1			0
#define SDXC_WIDTH4			1
#define SDXC_WIDTH8			2

/* smc command bits */
#define SDXC_RESP_EXPIRE		BIT(6)
#define SDXC_LONG_RESPONSE		BIT(7)
#define SDXC_CHECK_RESPONSE_CRC		BIT(8)
#define SDXC_DATA_EXPIRE		BIT(9)
#define SDXC_WRITE			BIT(10)
#define SDXC_SEQUENCE_MODE		BIT(11)
#define SDXC_SEND_AUTO_STOP		BIT(12)
#define SDXC_WAIT_PRE_OVER		BIT(13)
#define SDXC_STOP_ABORT_CMD		BIT(14)
#define SDXC_SEND_INIT_SEQUENCE		BIT(15)
#define SDXC_UPCLK_ONLY			BIT(21)
#define SDXC_READ_CEATA_DEV		BIT(22)
#define SDXC_CCS_EXPIRE			BIT(23)
#define SDXC_ENABLE_BIT_BOOT		BIT(24)
#define SDXC_ALT_BOOT_OPTIONS		BIT(25)
#define SDXC_BOOT_ACK_EXPIRE		BIT(26)
#define SDXC_BOOT_ABORT			BIT(27)
#define SDXC_VOLTAGE_SWITCH	        BIT(28)
#define SDXC_USE_HOLD_REGISTER	        BIT(29)
#define SDXC_START			BIT(31)

/* interrupt bits */
#define SDXC_RESP_ERROR			BIT(1)
#define SDXC_COMMAND_DONE		BIT(2)
#define SDXC_DATA_OVER			BIT(3)
#define SDXC_TX_DATA_REQUEST		BIT(4)
#define SDXC_RX_DATA_REQUEST		BIT(5)
#define SDXC_RESP_CRC_ERROR		BIT(6)
#define SDXC_DATA_CRC_ERROR		BIT(7)
#define SDXC_RESP_TIMEOUT		BIT(8)
#define SDXC_DATA_TIMEOUT		BIT(9)
#define SDXC_VOLTAGE_CHANGE_DONE	BIT(10)
#define SDXC_FIFO_RUN_ERROR		BIT(11)
#define SDXC_HARD_WARE_LOCKED		BIT(12)
#define SDXC_START_BIT_ERROR		BIT(13)
#define SDXC_AUTO_COMMAND_DONE		BIT(14)
#define SDXC_END_BIT_ERROR		BIT(15)
#define SDXC_SDIO_INTERRUPT		BIT(16)
#define SDXC_CARD_INSERT		BIT(30)
#define SDXC_CARD_REMOVE		BIT(31)
#define SDXC_INTERRUPT_ERROR_BIT \
	(SDXC_RESP_ERROR | SDXC_RESP_CRC_ERROR | SDXC_DATA_CRC_ERROR | \
	 SDXC_RESP_TIMEOUT | SDXC_DATA_TIMEOUT | SDXC_FIFO_RUN_ERROR | \
	 SDXC_HARD_WARE_LOCKED | SDXC_START_BIT_ERROR | SDXC_END_BIT_ERROR)
#define SDXC_INTERRUPT_DONE_BIT \
	(SDXC_AUTO_COMMAND_DONE | SDXC_DATA_OVER | \
	 SDXC_COMMAND_DONE | SDXC_VOLTAGE_CHANGE_DONE)

/* status */
#define SDXC_RXWL_FLAG			BIT(0)
#define SDXC_TXWL_FLAG			BIT(1)
#define SDXC_FIFO_EMPTY			BIT(2)
#define SDXC_FIFO_FULL			BIT(3)
#define SDXC_CARD_PRESENT		BIT(8)
#define SDXC_CARD_DATA_BUSY		BIT(9)
#define SDXC_DATA_FSM_BUSY		BIT(10)
#define SDXC_DMA_REQUEST		BIT(31)
#define SDXC_FIFO_SIZE			16

/* Function select */
#define SDXC_CEATA_ON			(0xceaa << 16)
#define SDXC_SEND_IRQ_RESPONSE		BIT(0)
#define SDXC_SDIO_READ_WAIT		BIT(1)
#define SDXC_ABORT_READ_DATA		BIT(2)
#define SDXC_SEND_CCSD			BIT(8)
#define SDXC_SEND_AUTO_STOPCCSD		BIT(9)
#define SDXC_CEATA_DEV_IRQ_ENABLE	BIT(10)

/* IDMA controller bus mod bit field */
#define SDXC_IDMAC_SOFT_RESET		BIT(0)
#define SDXC_IDMAC_FIX_BURST		BIT(1)
#define SDXC_IDMAC_IDMA_ON		BIT(7)
#define SDXC_IDMAC_REFETCH_DES		BIT(31)

/* IDMA status bit field */
#define SDXC_IDMAC_TRANSMIT_INTERRUPT		BIT(0)
#define SDXC_IDMAC_RECEIVE_INTERRUPT		BIT(1)
#define SDXC_IDMAC_FATAL_BUS_ERROR		BIT(2)
#define SDXC_IDMAC_DESTINATION_INVALID		BIT(4)
#define SDXC_IDMAC_CARD_ERROR_SUM		BIT(5)
#define SDXC_IDMAC_NORMAL_INTERRUPT_SUM		BIT(8)
#define SDXC_IDMAC_ABNORMAL_INTERRUPT_SUM	BIT(9)
#define SDXC_IDMAC_HOST_ABORT_INTERRUPT		BIT(10)
#define SDXC_IDMAC_IDLE				(0 << 13)
#define SDXC_IDMAC_SUSPEND			(1 << 13)
#define SDXC_IDMAC_DESC_READ			(2 << 13)
#define SDXC_IDMAC_DESC_CHECK			(3 << 13)
#define SDXC_IDMAC_READ_REQUEST_WAIT		(4 << 13)
#define SDXC_IDMAC_WRITE_REQUEST_WAIT		(5 << 13)
#define SDXC_IDMAC_READ				(6 << 13)
#define SDXC_IDMAC_WRITE			(7 << 13)
#define SDXC_IDMAC_DESC_CLOSE			(8 << 13)

/*
* If the idma-des-size-bits of property is ie 13, bufsize bits are:
*  Bits  0-12: buf1 size
*  Bits 13-25: buf2 size
*  Bits 26-31: not used
* Since we only ever set buf1 size, we can simply store it directly.
*/
#define SDXC_IDMAC_DES0_DIC	BIT(1)  /* disable interrupt on completion */
#define SDXC_IDMAC_DES0_LD	BIT(2)  /* last descriptor */
#define SDXC_IDMAC_DES0_FD	BIT(3)  /* first descriptor */
#define SDXC_IDMAC_DES0_CH	BIT(4)  /* chain mode */
#define SDXC_IDMAC_DES0_ER	BIT(5)  /* end of ring */
#define SDXC_IDMAC_DES0_CES	BIT(30) /* card error summary */
#define SDXC_IDMAC_DES0_OWN	BIT(31) /* 1-idma owns it, 0-host owns it */

struct sunxi_idma_des {
	u32	config;
	u32	buf_size;
	u32	buf_addr_ptr1;
	u32	buf_addr_ptr2;
};

struct sunxi_mmc_host {
	struct mmc_host	*mmc;
	struct regulator *vmmc;
	struct reset_control *reset;

	/* IO mapping base */
	void __iomem	*reg_base;

	spinlock_t	lock;
	struct tasklet_struct manual_stop_tasklet;

	/* clock management */
	struct clk	*clk_ahb;
	struct clk	*clk_mod;

	/* ios information */
	u32		clk_mod_rate;
	u32		bus_width;
	u32		idma_des_size_bits;
	u32		ddr;
	u32		voltage_switching;

	/* irq */
	int		irq;
	u32		int_sum;
	u32		sdio_imask;

	/* flags */
	bool		wait_dma;

	dma_addr_t	sg_dma;
	void		*sg_cpu;

	struct mmc_request *mrq;
	struct mmc_request *manual_stop_mrq;
	u32		ferror;
};

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

	if (smc_host->reset) {
		ret = reset_control_deassert(smc_host->reset);
		if (ret) {
			dev_err(mmc_dev(smc_host->mmc), "reset err %d\n", ret);
			clk_disable_unprepare(smc_host->clk_ahb);
			clk_disable_unprepare(smc_host->clk_mod);
			return ret;
		}
	}

	/* reset controller */
	rval = mci_readl(smc_host, REG_GCTRL);
	rval |= SDXC_HARDWARE_RESET;
	mci_writel(smc_host, REG_GCTRL, rval);

	mci_writel(smc_host, REG_FTRGL, 0x20070008);
	mci_writel(smc_host, REG_TMOUT, 0xffffffff);
	mci_writel(smc_host, REG_IMASK, smc_host->sdio_imask);
	mci_writel(smc_host, REG_RINTR, 0xffffffff);
	mci_writel(smc_host, REG_DBGC, 0xdeb);
	mci_writel(smc_host, REG_FUNS, SDXC_CEATA_ON);
	mci_writel(smc_host, REG_DLBA, smc_host->sg_dma);

	rval = mci_readl(smc_host, REG_GCTRL);
	rval |= SDXC_INTERRUPT_ENABLE_BIT;
	rval &= ~SDXC_ACCESS_DONE_DIRECT;
	mci_writel(smc_host, REG_GCTRL, rval);

	return 0;
}

static void sunxi_mmc_exit_host(struct sunxi_mmc_host *smc_host)
{
	mci_writel(smc_host, REG_GCTRL, SDXC_HARDWARE_RESET);

	if (smc_host->reset)
		reset_control_assert(smc_host->reset);

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

	/*
	 * Avoid the io-store starting the idmac hitting io-mem before the
	 * descriptors hit the main-mem.
	 */
	wmb();
}

static enum dma_data_direction sunxi_mmc_get_dma_dir(struct mmc_data *data)
{
	if (data->flags & MMC_DATA_WRITE)
		return DMA_TO_DEVICE;
	else
		return DMA_FROM_DEVICE;
}

static int sunxi_mmc_map_dma(struct sunxi_mmc_host *smc_host,
			     struct mmc_data *data)
{
	u32 i, dma_len;
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

	return 0;
}

static void sunxi_mmc_start_dma(struct sunxi_mmc_host *smc_host,
				struct mmc_data *data)
{
	u32 rval;

	sunxi_mmc_init_idma_des(smc_host, data);

	rval = mci_readl(smc_host, REG_GCTRL);
	rval |= SDXC_DMA_ENABLE_BIT;
	mci_writel(smc_host, REG_GCTRL, rval);
	rval |= SDXC_DMA_RESET;
	mci_writel(smc_host, REG_GCTRL, rval);

	mci_writel(smc_host, REG_DMAC, SDXC_IDMAC_SOFT_RESET);

	if (!(data->flags & MMC_DATA_WRITE))
		mci_writel(smc_host, REG_IDIE, SDXC_IDMAC_RECEIVE_INTERRUPT);

	mci_writel(smc_host, REG_DMAC,
		   SDXC_IDMAC_FIX_BURST | SDXC_IDMAC_IDMA_ON);
}

static void sunxi_mmc_send_manual_stop(struct sunxi_mmc_host *host,
				       struct mmc_request *req)
{
	u32 cmd_val = SDXC_START | SDXC_RESP_EXPIRE | SDXC_STOP_ABORT_CMD
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
	if ((smc_host->int_sum & SDXC_INTERRUPT_ERROR_BIT) ==
		SDXC_RESP_TIMEOUT && (cmd->opcode == SD_IO_SEND_OP_COND ||
				      cmd->opcode == SD_IO_RW_DIRECT))
		return;

	dev_err(mmc_dev(smc_host->mmc),
		"smc %d err, cmd %d,%s%s%s%s%s%s%s%s%s%s !!\n",
		smc_host->mmc->index, cmd->opcode,
		data ? (data->flags & MMC_DATA_WRITE ? " WR" : " RD") : "",
		smc_host->int_sum & SDXC_RESP_ERROR     ? " RE"     : "",
		smc_host->int_sum & SDXC_RESP_CRC_ERROR  ? " RCE"    : "",
		smc_host->int_sum & SDXC_DATA_CRC_ERROR  ? " DCE"    : "",
		smc_host->int_sum & SDXC_RESP_TIMEOUT ? " RTO"    : "",
		smc_host->int_sum & SDXC_DATA_TIMEOUT ? " DTO"    : "",
		smc_host->int_sum & SDXC_FIFO_RUN_ERROR  ? " FE"     : "",
		smc_host->int_sum & SDXC_HARD_WARE_LOCKED ? " HL"     : "",
		smc_host->int_sum & SDXC_START_BIT_ERROR ? " SBE"    : "",
		smc_host->int_sum & SDXC_END_BIT_ERROR   ? " EBE"    : ""
		);
}

/* Called in interrupt context! */
static int sunxi_mmc_finalize_request(struct sunxi_mmc_host *host)
{
	struct mmc_request *mrq = host->mrq;

	mci_writel(host, REG_IMASK, host->sdio_imask);
	mci_writel(host, REG_IDIE, 0);

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
		u32 rval;

		mci_writel(host, REG_IDST, 0x337);
		mci_writel(host, REG_DMAC, 0);
		rval = mci_readl(host, REG_GCTRL);
		rval |= SDXC_DMA_RESET;
		mci_writel(host, REG_GCTRL, rval);
		rval &= ~SDXC_DMA_ENABLE_BIT;
		mci_writel(host, REG_GCTRL, rval);
		rval |= SDXC_FIFO_RESET;
		mci_writel(host, REG_GCTRL, rval);
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				     sunxi_mmc_get_dma_dir(data));
	}

	mci_writel(host, REG_RINTR, 0xffff);

	dev_dbg(mmc_dev(host->mmc), "req done, resp %08x %08x %08x %08x\n",
		mrq->cmd->resp[0], mrq->cmd->resp[1],
		mrq->cmd->resp[2], mrq->cmd->resp[3]);

	host->mrq = NULL;
	host->int_sum = 0;
	host->wait_dma = false;

	if (mrq->data && mrq->data->error) {
		host->manual_stop_mrq = mrq;
		tasklet_schedule(&host->manual_stop_tasklet);
		return -EBUSY;
	}

	return 0;
}

static irqreturn_t sunxi_mmc_irq(int irq, void *dev_id)
{
	struct sunxi_mmc_host *host = dev_id;
	struct mmc_request *mrq;
	bool finalize = false;
	bool complete = false;
	bool sdio_int = false;
	u32 msk_int;
	u32 idma_int;

	spin_lock(&host->lock);

	idma_int  = mci_readl(host, REG_IDST);
	msk_int   = mci_readl(host, REG_MISTA);

	dev_dbg(mmc_dev(host->mmc), "irq: rq %p mi %08x idi %08x\n",
		host->mrq, msk_int, idma_int);

	mrq = host->mrq;
	if (mrq) {
		if (idma_int & SDXC_IDMAC_RECEIVE_INTERRUPT)
			host->wait_dma = false;

		host->int_sum |= msk_int;

		/* Wait for COMMAND_DONE on RESPONSE_TIMEOUT before finalize */
		if ((host->int_sum & SDXC_RESP_TIMEOUT) &&
				!(host->int_sum & SDXC_COMMAND_DONE))
			mci_writel(host, REG_IMASK,
				   host->sdio_imask | SDXC_COMMAND_DONE);
		/* Don't wait for dma on error */
		else if (host->int_sum & SDXC_INTERRUPT_ERROR_BIT)
			finalize = true;
		else if ((host->int_sum & SDXC_INTERRUPT_DONE_BIT) &&
				!host->wait_dma)
			finalize = true;
	}

	if (msk_int & SDXC_SDIO_INTERRUPT)
		sdio_int = true;

	mci_writel(host, REG_RINTR, msk_int);
	mci_writel(host, REG_IDST, idma_int);

	if (finalize) {
		if (sunxi_mmc_finalize_request(host) == 0)
			complete = true;
	}

	spin_unlock(&host->lock);

	if (complete)
		mmc_request_done(host->mmc, mrq);

	if (sdio_int)
		mmc_signal_sdio_irq(host->mmc);

	return IRQ_HANDLED;
}

static void sunxi_mmc_manual_stop_tasklet(unsigned long data)
{
	struct sunxi_mmc_host *host = (struct sunxi_mmc_host *) data;
	struct mmc_request *mrq;
	unsigned long iflags;

	spin_lock_irqsave(&host->lock, iflags);
	mrq = host->manual_stop_mrq;
	spin_unlock_irqrestore(&host->lock, iflags);

	if (!mrq) {
		dev_err(mmc_dev(host->mmc), "no request for manual stop\n");
		return;
	}

	dev_err(mmc_dev(host->mmc), "data error, sending stop command\n");
	sunxi_mmc_send_manual_stop(host, mrq);

	spin_lock_irqsave(&host->lock, iflags);
	host->manual_stop_mrq = NULL;
	spin_unlock_irqrestore(&host->lock, iflags);

	mmc_request_done(host->mmc, mrq);
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

static void sunxi_mmc_clk_set_rate(struct sunxi_mmc_host *smc_host,
				   unsigned int rate)
{
	u32 newrate, oclk_dly, rval, sclk_dly, src_clk;
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
	clk_set_rate(smc_host->clk_mod, newrate);
	smc_host->clk_mod_rate = clk_get_rate(smc_host->clk_mod);
	dev_dbg(mmc_dev(smc_host->mmc), "clk is now %d\n",
		smc_host->clk_mod_rate);

	sunxi_mmc_oclk_onoff(smc_host, 0);
	/* clear internal divider */
	rval = mci_readl(smc_host, REG_CLKCR);
	rval &= ~0xff;
	mci_writel(smc_host, REG_CLKCR, rval);

	/* determine delays */
	if (rate <= 400000) {
		oclk_dly = 0;
		sclk_dly = 7;
	} else if (rate <= 25000000) {
		oclk_dly = 0;
		sclk_dly = 5;
	} else if (rate <= 50000000) {
		if (smc_host->ddr) {
			oclk_dly = 2;
			sclk_dly = 4;
		} else {
			oclk_dly = 3;
			sclk_dly = 5;
		}
	} else {
		/* rate > 50000000 */
		oclk_dly = 2;
		sclk_dly = 4;
	}

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
	u32 rval;
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
	rval = mci_readl(host, REG_GCTRL);
	if (ios->timing == MMC_TIMING_UHS_DDR50) {
		rval |= SDXC_DDR_MODE;
		host->ddr = 1;
	} else {
		rval &= ~SDXC_DDR_MODE;
		host->ddr = 0;
	}
	mci_writel(host, REG_GCTRL, rval);

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
	int ret;

	if (!mmc_gpio_get_cd(mmc) || host->ferror) {
		dev_dbg(mmc_dev(host->mmc), "no medium present\n");
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	if (data) {
		ret = sunxi_mmc_map_dma(host, data);
		if (ret < 0) {
			dev_err(mmc_dev(host->mmc), "map DMA failed\n");
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
		cmd_val |= SDXC_RESP_EXPIRE;
		if (cmd->flags & MMC_RSP_136)
			cmd_val |= SDXC_LONG_RESPONSE;
		if (cmd->flags & MMC_RSP_CRC)
			cmd_val |= SDXC_CHECK_RESPONSE_CRC;

		if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
			cmd_val |= SDXC_DATA_EXPIRE | SDXC_WAIT_PRE_OVER;
			if (cmd->data->flags & MMC_DATA_STREAM) {
				imask |= SDXC_AUTO_COMMAND_DONE;
				cmd_val |= SDXC_SEQUENCE_MODE |
					   SDXC_SEND_AUTO_STOP;
			}

			if (cmd->data->stop) {
				imask |= SDXC_AUTO_COMMAND_DONE;
				cmd_val |= SDXC_SEND_AUTO_STOP;
			} else {
				imask |= SDXC_DATA_OVER;
			}

			if (cmd->data->flags & MMC_DATA_WRITE)
				cmd_val |= SDXC_WRITE;
			else
				host->wait_dma = true;
		} else {
			imask |= SDXC_COMMAND_DONE;
		}
	} else {
		imask |= SDXC_COMMAND_DONE;
	}

	dev_dbg(mmc_dev(host->mmc), "cmd %d(%08x) arg %x ie 0x%08x len %d\n",
		cmd_val & 0x3f, cmd_val, cmd->arg, imask,
		mrq->data ? mrq->data->blksz * mrq->data->blocks : 0);

	spin_lock_irqsave(&host->lock, iflags);

	if (host->mrq || host->manual_stop_mrq) {
		spin_unlock_irqrestore(&host->lock, iflags);

		if (data)
			dma_unmap_sg(mmc_dev(host->mmc), data->sg,
				data->sg_len, sunxi_mmc_get_dma_dir(data));

		dev_err(mmc_dev(host->mmc), "request already pending\n");
		mrq->cmd->error = -EBUSY;
		mmc_request_done(host->mmc, mrq);
		return;
	}

	if (data) {
		mci_writel(host, REG_BLKSZ, data->blksz);
		mci_writel(host, REG_BCNTR, data->blksz * data->blocks);
		sunxi_mmc_start_dma(host, data);
	}

	host->mrq = mrq;
	mci_writel(host, REG_IMASK, host->sdio_imask | imask);
	mci_writel(host, REG_CARG, cmd->arg);
	mci_writel(host, REG_CMDR, cmd_val);

	spin_unlock_irqrestore(&host->lock, iflags);
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

	host->reset = devm_reset_control_get(&pdev->dev, "ahb");
	if (IS_ERR(host->reset))
		host->reset = NULL; /* Having a reset controller is optional */

	/*
	 * Sometimes the controller asserts the irq on boot for some reason,
	 * and since it is not clocked there is no way to clear it. So make
	 * sure the controller is in a sane state before enabling irqs.
	 */
	ret = sunxi_mmc_init_host(host->mmc);
	if (ret)
		return ret;

	host->irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, host->irq, sunxi_mmc_irq, 0,
			       "sunxi-mmc", host);
	if (ret == 0)
		disable_irq(host->irq);

	/* And disable the controller again */
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
	tasklet_init(&host->manual_stop_tasklet,
		     sunxi_mmc_manual_stop_tasklet, (unsigned long)host);

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

	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;
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
	tasklet_disable(&host->manual_stop_tasklet);
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
