/*
 * Copyright (C) 2014
 * Chen-Yu Tsai <wens@csie.org>
 *
 * Based on ux500.c and Allwinner driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/usb/of.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/sun4i-sc.h>

#include "musb_core.h"

#define ISCR_ID_PULLUP_EN              BIT(17)
#define ISCR_DPDM_PULLUP_EN            BIT(16)
#define ISCR_FORCE_ID_MASK             (0x3 << 14)
#define ISCR_FORCE_ID_LOW              (0x2 << 14)
#define ISCR_FORCE_ID_HIGH             (0x3 << 14)
#define ISCR_FORCE_VBUS_VALID_MASK     (0x3 << 12)
#define ISCR_FORCE_VBUS_VALID_LOW      (0x2 << 12)
#define ISCR_FORCE_VBUS_VALID_HIGH     (0x3 << 12)

/* Allwinner OTG supports up to 5 endpoints */
#define SUNXI_MUSB_MAX_EP_NUM	6
#define SUNXI_MUSB_RAM_BITS	11

static struct musb_fifo_cfg sunxi_musb_mode_cfg[] = {
	MUSB_EP_FIFO_SINGLE(1, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(1, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_RX, 512),
};

static struct musb_hdrc_config sunxi_musb_hdrc_config = {
	.fifo_cfg       = sunxi_musb_mode_cfg,
	.fifo_cfg_size  = ARRAY_SIZE(sunxi_musb_mode_cfg),
	.multipoint	= true,
	.dyn_fifo	= true, /* deprecated, but was in Allwinner driver */
	.soft_con       = true, /* deprecated, but was in Allwinner driver */
	.num_eps	= SUNXI_MUSB_MAX_EP_NUM,
	.ram_bits	= SUNXI_MUSB_RAM_BITS,
};

struct sunxi_glue {
	struct device		*dev;
	struct platform_device	*musb;
	struct clk		*clk;
	struct regulator	*vbus;
	struct regmap		*sc;
	int			id_det_gpio;
	int			vbus_gpio;
	int			id_det_irq;
	int			vbus_irq;
};

#define glue_to_musb(g)	platform_get_drvdata(g->musb)

static void sunxi_musb_set_vbus(struct musb *musb, int is_on)
{
	u8            devctl;
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		if (musb->xceiv->state == OTG_STATE_A_IDLE) {
			/* start the session */
			devctl |= MUSB_DEVCTL_SESSION;
			musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
			/*
			 * Wait for the musb to set as A device to enable the
			 * VBUS
			 */
			while (musb_readb(musb->mregs, MUSB_DEVCTL) & 0x80) {

				if (time_after(jiffies, timeout)) {
					dev_err(musb->controller,
					"configured as A device timeout");
					break;
				}
			}

		} else {
			musb->is_active = 1;
			musb->xceiv->otg->default_a = 1;
			musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
			devctl |= MUSB_DEVCTL_SESSION;
			MUSB_HST_MODE(musb);
		}
	} else {
		musb->is_active = 0;

		/* NOTE: we're skipping A_WAIT_VFALL -> A_IDLE and jumping
		 * right to B_IDLE...
		 */
		musb->xceiv->otg->default_a = 0;
		devctl &= ~MUSB_DEVCTL_SESSION;
		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	/*
	 * Devctl values will be updated after vbus goes below
	 * session_valid. The time taken depends on the capacitance
	 * on VBUS line. The max discharge time can be upto 1 sec
	 * as per the spec. Typically on our platform, it is 200ms
	 */
	if (!is_on)
		mdelay(200);

	dev_dbg(musb->controller, "VBUS %s, devctl %02x\n",
		usb_otg_state_string(musb->xceiv->state),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

static irqreturn_t sunxi_musb_interrupt(int irq, void *__hci)
{
	unsigned long   flags;
	irqreturn_t     retval = IRQ_NONE;
	struct musb     *musb = __hci;

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval = musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	return retval;
}

static int sunxi_musb_init(struct musb *musb)
{
	struct device *dev = musb->controller;
	int res;

	musb->phy = devm_phy_get(dev->parent, "usb");
	if (IS_ERR(musb->phy)) {
		if (PTR_ERR(musb->phy) == -EPROBE_DEFER) {
			dev_info(dev, "phy probe deferred\n");
			return -EPROBE_DEFER;
		}
		dev_err(dev, "no phy configured\n");
		return PTR_ERR(musb->phy);
	}

	musb->xceiv = devm_usb_get_phy_by_phandle(dev->parent,"usb-phy", 0);
	if (IS_ERR(musb->xceiv)) {
		if (PTR_ERR(musb->xceiv) == -EPROBE_DEFER) {
			dev_info(dev, "usb phy probe deferred\n");
			return -EPROBE_DEFER;
		}
		dev_err(dev, "no usb phy configured\n");
		return PTR_ERR(musb->xceiv);
	}

	res = phy_init(musb->phy);
	if (res)
		return res;

	res = usb_phy_init(musb->xceiv);
	if (res)
		return res;

	musb->isr = sunxi_musb_interrupt;

	return 0;
}

static int sunxi_musb_exit(struct musb *musb)
{
	return 0;
}

static void sunxi_musb_enable(struct musb *musb)
{
}

static void sunxi_musb_disable(struct musb *musb)
{
}

static const struct musb_platform_ops sunxi_ops = {
	.init		= sunxi_musb_init,
	.exit		= sunxi_musb_exit,

	.enable		= sunxi_musb_enable,
	.disable	= sunxi_musb_disable,

	.set_vbus	= sunxi_musb_set_vbus,
};

static struct musb_hdrc_platform_data *
sunxi_musb_of_probe(struct platform_device *pdev, struct device_node *np)
{
	struct musb_hdrc_platform_data *pdata;
	int mode;

	return pdata;
}

static int sunxi_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata;
	struct platform_device		*musb;
	struct device_node		*np = pdev->dev.of_node;
	struct sunxi_glue		*glue;
	struct resource			*res, musb_resources[2];
	struct regmap			*sc;
	struct clk			*clk;
	int				gpio, ret = -ENOMEM;
	u32				flags;

	if (!np) {
		dev_err(&pdev->dev, "no device tree node found\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	mode = of_usb_get_dr_mode(np);

	switch (mode) {
		case USB_DR_MODE_HOST:
			pdata->mode = MUSB_HOST;
			break;

		case USB_DR_MODE_PERIPHERAL:
			pdata->mode = MUSB_PERIPHERAL;
			break;

		case USB_DR_MODE_OTG:
			pdata->mode = MUSB_OTG;
			break;

		case USB_DR_MODE_UNKNOWN:
		default:
			dev_err(&pdev->dev, "No 'dr_mode' property found\n");
			goto err0;
	}

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pdev->dev, "failed to allocate glue context\n");
		return -ENOMEM;
	}

	gpio = of_get_named_gpio_flags(np, "id_det-gpios", 0, &flags);
	if (gpio < 0)
		return -EINVAL;
	glue->id_det_gpio = gpio;

	gpio = of_get_named_gpio_flags(np, "vbus-gpios", 0, &flags);
	if (gpio < 0)
		return -EINVAL;
	glue->vbus_gpio = gpio;

	sc = syscon_regmap_lookup_by_phandle(np, "allwinner,syscon");
	if (IS_ERR(sc)) {
		ret = PTR_ERR(sc);
		goto err0;
	}

	ret = regmap_update_bits(sc, 0, BIT(0), BIT(0));
	if (ret) {
		dev_err(&pdev->dev, "failed to set SRAM mapping\n");
		goto err0;
	}

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(clk);
		goto err2;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err3;
	}

	musb = platform_device_alloc("musb-hdrc", PLATFORM_DEVID_AUTO);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err3;
	}

	glue->dev			= &pdev->dev;
	glue->musb			= musb;
	glue->clk			= clk;
	glue->sc			= sc;

	pdata->platform_ops		= &sunxi_ops;
	pdata->config 			= &sunxi_musb_hdrc_config;

	musb->dev.parent		= &pdev->dev;
	musb->dev.dma_mask		= &pdev->dev.coherent_dma_mask;
	musb->dev.coherent_dma_mask	= pdev->dev.coherent_dma_mask;

	platform_set_drvdata(pdev, glue);

	memset(musb_resources, 0, sizeof(*musb_resources) *
			ARRAY_SIZE(musb_resources));

	musb_resources[0].name = pdev->resource[0].name;
	musb_resources[0].start = pdev->resource[0].start;
	musb_resources[0].end = pdev->resource[0].end;
	musb_resources[0].flags = pdev->resource[0].flags;

	musb_resources[1].name = pdev->resource[2].name;
	musb_resources[1].start = pdev->resource[2].start;
	musb_resources[1].end = pdev->resource[2].end;
	musb_resources[1].flags = pdev->resource[2].flags;

	ret = platform_device_add_resources(musb, musb_resources,
			ARRAY_SIZE(musb_resources));
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err5;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err5;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err5;
	}

	return 0;

err5:
	platform_device_put(musb);

err3:
	clk_disable_unprepare(clk);

err2:
	regmap_update_bits(sc, 0, BIT(0), 0);

err0:
	return ret;
}

static int sunxi_remove(struct platform_device *pdev)
{
	struct sunxi_glue	*glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);
	clk_disable_unprepare(glue->clk);
	regmap_update_bits(glue->sc, 0, BIT(0), 0);

	return 0;
}

#ifdef CONFIG_PM
static int sunxi_suspend(struct device *dev)
{
	struct sunxi_glue	*glue = dev_get_drvdata(dev);
	struct musb		*musb = glue_to_musb(glue);

	usb_phy_set_suspend(musb->xceiv, 1);
	clk_disable_unprepare(glue->clk);

	return 0;
}

static int sunxi_resume(struct device *dev)
{
	struct sunxi_glue	*glue = dev_get_drvdata(dev);
	struct musb		*musb = glue_to_musb(glue);
	int			ret;

	ret = clk_prepare_enable(glue->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}

	usb_phy_set_suspend(musb->xceiv, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sunxi_pm_ops, sunxi_suspend, sunxi_resume);

static const struct of_device_id sunxi_match[] = {
	{ .compatible = "allwinner,sun4i-a10-musb", },
	{}
};

static struct platform_driver sunxi_driver = {
	.probe		= sunxi_probe,
	.remove		= sunxi_remove,
	.driver		= {
		.name	= "musb-sunxi",
		.pm	= &sunxi_pm_ops,
		.of_match_table = sunxi_match,
	},
};

MODULE_DESCRIPTION("Allwinner sunxi MUSB Glue Layer");
MODULE_AUTHOR("Chen-Yu Tsai <wens@csie.org>");
MODULE_LICENSE("GPL v2");
module_platform_driver(sunxi_driver);
