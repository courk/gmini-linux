/*
 * linux/drivers/usb/host/ehci-berlin.c
 *
 * Authors: Hongzhan Chen <hzchen@marvell.com>
 * Copyright (C) 2011 Marvell Ltd.
 *
 * Based on "ehci-au1xxx.c"
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <mach/gpio.h>
#include <mach/galois_platform.h>

#define MV_USB_CHIP_ID			0
#define  MV_USB_CHIP_ID_MASK		0x3F
#define  MV_USB_CHIP_USB20		0x05
#define MV_SBUSCFG			0x090
#define MV_USB_CHIP_CAP			0x100
#define MV_USBMODE			0x1a8
#define  USBMODE_CM_HOST		3

#define MV_USB_PHY_PLL_REG		0x04
#define MV_USB_PHY_PLL_CONTROL_REG	0x08
#define MV_USB_PHY_ANALOG_REG		0x34
#define MV_USB_PHY_RX_CTRL_REG		0x20
#define PHY_PLL	0x54C0

#define MV_USB_RESET_TRIGGER		0x0178	/* RA_Gbl_ResetTrigger */

#define USB2_OTG_REG0		0x34
#define USB2_CHARGER_REG0	0x38
#define USB2_PLL_REG0		0x0
#define USB2_PLL_REG1		0x4
#define USB2_DIG_REG0		0x1c
#define USB2_TX_CH_CTRL0	0x0c
#define USB2_CAL_CTRL		8

struct berlin_ehci_hcd {
	struct ehci_hcd *ehci;
	void __iomem *phy_base;
	int reset;
	int pwr_gpio;
};

static int berlin_cdp_reinit_phy(void __iomem *base)
{
	u32 data, timeout;

	/* powering up OTG */
	data = readl( base + USB2_OTG_REG0);
	data |= 1<<4;
	writel(data, (base + USB2_OTG_REG0));

	/* powering Charger detector circuit */
	data = readl( base + USB2_CHARGER_REG0);
	data |= 1<<5;
	writel(data, (base + USB2_CHARGER_REG0));

	/* Power up analog port */
	writel(0x03BE7F6F, (base + USB2_TX_CH_CTRL0));

	/* Squelch setting */
	writel(0xC39F16CE, (base + USB2_DIG_REG0));

	/* Impedance calibration */
	writel(0xf5930488, (base + USB2_CAL_CTRL));

	/* Configuring FBDIV for SOC Clk 25 Mhz */
	data = readl( base + USB2_PLL_REG0);
	data &= 0xce00ff80;
	data |= 5 | (0x60<<16) | (0<<28);
	writel(data, (base + USB2_PLL_REG0));

	/* Power up PLL, Reset, Suspen_en disable */
	writel(0x407, (base + USB2_PLL_REG1));
	udelay(100);

	/* Deassert Reset */
	writel(0x403, (base + USB2_PLL_REG1));

	/* Wait for PLL Lock */
	timeout = 0x1000000;
	do {
		data = readl( base + USB2_PLL_REG0);
		if (!--timeout)
			break;
	} while ( !(data&0x80000000));

	if (!timeout)
		printk(KERN_ERR "ERROR: USB PHY PLL NOT LOCKED!\n");

	return 0;
}

static void mv_start_ehc(struct usb_hcd *hcd)
{
	struct berlin_ehci_hcd *berlin = dev_get_drvdata(hcd->self.controller);

	if (berlin->pwr_gpio >= 0)
		GPIO_PortWrite(berlin->pwr_gpio, 0);
#ifdef CONFIG_BERLIN2CDP
	berlin_cdp_reinit_phy(berlin->phy_base);
#else
	writel(PHY_PLL, berlin->phy_base + MV_USB_PHY_PLL_REG);
	writel(0x2235, berlin->phy_base + MV_USB_PHY_PLL_CONTROL_REG);
	writel(0x5680, berlin->phy_base + MV_USB_PHY_ANALOG_REG);
	writel(0xAA79, berlin->phy_base + MV_USB_PHY_RX_CTRL_REG);

	if (berlin->reset >= 0) {
		u32 temp;
		temp = 1 << berlin->reset;
		writel(temp, IOMEM(MEMMAP_CHIP_CTRL_REG_BASE + MV_USB_RESET_TRIGGER));
	}
#endif
	if (berlin->pwr_gpio >= 0)
		GPIO_PortWrite(berlin->pwr_gpio, 1);
}

static void mv_stop_ehc(struct usb_hcd *hcd)
{
	struct berlin_ehci_hcd *berlin = dev_get_drvdata(hcd->self.controller);

	if (berlin->pwr_gpio >= 0)
		GPIO_PortWrite(berlin->pwr_gpio, 0);
}

static int ehci_mv_setup(struct usb_hcd *hcd)
{
	mv_start_ehc(hcd);
	hcd->has_tt = 1;
	return ehci_setup(hcd);
}

static int ehci_mv_start( struct usb_hcd *hcd)
{
	int retval;
	struct ehci_hcd  *ehci = hcd_to_ehci(hcd);

	retval = ehci_run(hcd);

	ehci_writel(ehci, 0x07, hcd->regs + MV_SBUSCFG);
	ehci_writel(ehci, 0x13, hcd->regs + MV_USBMODE);

	return retval;

}

static const struct hc_driver ehci_mv_hc_driver = {
	.description = hcd_name,
	.product_desc = "Marvell Berlin SoC EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_mv_setup,
	.start = ehci_mv_start,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,
	.endpoint_reset = ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,

	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

static int berlin_ehci_probe(struct platform_device *pdev)
{
	int retval, irq;
	u32 val;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	struct resource *res;
	struct berlin_ehci_hcd *berlin;
	struct device_node *np = pdev->dev.of_node;

	if (usb_disabled()){
		return -ENODEV;
	}

	berlin = devm_kzalloc(&pdev->dev, sizeof(*berlin), GFP_KERNEL);
	if (!berlin)
		return -ENOMEM;

	if (of_property_read_u32(np, "phy-base", &val)) {
		dev_err(&pdev->dev, "no phy base set\n");
		return -EINVAL;
	}
	berlin->phy_base = (void __iomem *)val;

	if (of_property_read_u32(np, "reset-bit", &val)) {
		/* berlin2cdp don't need reset */
		berlin->reset = -1;
	} else {
		berlin->reset = val;
	}

	if (of_property_read_u32(np, "pwr-gpio", &val)) {
		/* some platform don't have pwr gpio */
		berlin->pwr_gpio = -1;
	} else {
		berlin->pwr_gpio = val;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	hcd = usb_create_hcd(&ehci_mv_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		retval = -ENOMEM;
		goto err1;
	}

	platform_set_drvdata(pdev, berlin);

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		retval = -EBUSY;
		goto err2;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);

	if (!hcd->regs) {
		dev_err(&pdev->dev, "ioremap failed \n");
		retval = -EFAULT;
		goto err3;
	}

	berlin->ehci = ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs + MV_USB_CHIP_CAP;
	ehci->regs = hcd->regs + MV_USB_CHIP_CAP + HC_LENGTH(ehci, readl(&ehci->caps->hc_capbase));

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);

	if (retval == 0) {
		dev_info(&pdev->dev, "usb_add_hcd successful\n");
		return retval;
	}

	iounmap(hcd->regs);
err3:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err2:
	usb_put_hcd(hcd);
err1:
	return retval;
}

static int berlin_ehci_remove(struct platform_device *pdev)
{
	struct berlin_ehci_hcd *berlin = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(berlin->ehci);

	mv_stop_ehc(hcd);
	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	return 0;
}

static void berlin_ehci_shutdown(struct platform_device *pdev)
{
	struct berlin_ehci_hcd *berlin = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(berlin->ehci);

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static const struct of_device_id berlin_ehci_of_match[] = {
	{.compatible = "mrvl,berlin-ehci",},
	{},
};
MODULE_DEVICE_TABLE(of, berlin_ehci_of_match);
MODULE_ALIAS("berlin-ehci");

static struct platform_driver berlin_ehci_driver = {
	.probe 		= berlin_ehci_probe,
	.remove 	= berlin_ehci_remove,
	.shutdown 	= berlin_ehci_shutdown,
	.driver = {
		.name = "berlin-ehci",
		.of_match_table = berlin_ehci_of_match,
	}
};
