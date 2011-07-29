#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdhci-pltfm.h>
#include "sdhci.h"
#include "sdhci-pltfm.h"

static u8 prs505_sdhci_readb(struct sdhci_host *host, int reg)
{
	u16 tmp;

	tmp = readw(host->ioaddr + (reg & ~(1U)));

	if (reg % 2)
		return (u8)(tmp >> 8);
	else
		return (u8)tmp;
}

static void prs505_sdhci_writeb(struct sdhci_host *host, u8 val, int reg)
{
	u16 tmp;
	int shift;
	int addr = reg & ~(1U);

	if (reg % 2)
		shift = 8;
	else
		shift = 0;

	tmp = readw(host->ioaddr + addr);

	tmp &= ~(0xff << shift);
	tmp |= (val << shift);

	writew(tmp, host->ioaddr + addr);
}

static void prs505_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	writew(val & 0xffff, host->ioaddr + reg);
	writew(val >> 16, host->ioaddr + reg + 2);
}

static u32 prs505_sdhci_readl(struct sdhci_host *host, int reg)
{
	u32 tmp;

	tmp = readw(host->ioaddr + reg);
	tmp |= readw(host->ioaddr + reg + 2) << 16;

	return tmp;
}

static struct sdhci_ops prs505_sdhci_ops = {
	.read_b		= prs505_sdhci_readb,
	.read_l		= prs505_sdhci_readl,
	.write_b	= prs505_sdhci_writeb,
	.write_l	= prs505_sdhci_writel,
};

struct sdhci_pltfm_data sdhci_r5c807_prs505_pdata = {
	.ops	= &prs505_sdhci_ops,
	.quirks	= SDHCI_QUIRK_BROKEN_BLOCK_PIO_READ | SDHCI_QUIRK_NO_MULTIBLOCK,
};

