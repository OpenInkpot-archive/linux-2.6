/*
 * i.MX Power Management
 *
 * Copyright (c) 2011 Alexander Kerner <lunohod@openinkpot.org>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <mach/system.h>
#include <mach/mx1.h>

static int mx1_suspend_enter(suspend_state_t state)
{
	u32 cscr;
	switch (state) {
	case PM_SUSPEND_MEM:
		/* Clear MPEN and SPEN to disable MPLL/SPLL */
		cscr = __raw_readl(MX1_IO_ADDRESS(MX1_CCM_BASE_ADDR));
		cscr &= 0xFFFFFFFC;
		__raw_writel(cscr, MX1_IO_ADDRESS(MX1_CCM_BASE_ADDR));
		/* Executes WFI */
		arch_idle();
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_MACH_SONY_PRS505
static int prs505_prepare_quirk(void)
{
	unsigned long timeout = jiffies + HZ / 5;
	unsigned int temp;

	/* Wait for UART1 RTS deassertion */
	do {
		temp = __raw_readl(MX1_IO_ADDRESS(MX1_UART1_BASE_ADDR + 0x94));
	} while ((temp & (1 << 14)) && time_before(jiffies, timeout));

	if (temp & (1 << 14))
		return -EBUSY;

	/* Clear RTSD interrupt flag */
	__raw_writel((1 << 12), MX1_IO_ADDRESS(MX1_UART1_BASE_ADDR + 0x94));

	return 0;
}
#endif

static int mx1_prepare(void)
{
	int ret = 0;

#ifdef CONFIG_MACH_SONY_PRS505
	ret = prs505_prepare_quirk();
#endif

	return ret;
}

static struct platform_suspend_ops mx1_suspend_ops = {
	.enter = mx1_suspend_enter,
	.valid = suspend_valid_only_mem,
	.prepare = mx1_prepare,
};

static int __init mx1_pm_init(void)
{
	suspend_set_ops(&mx1_suspend_ops);
	return 0;
}

device_initcall(mx1_pm_init);
