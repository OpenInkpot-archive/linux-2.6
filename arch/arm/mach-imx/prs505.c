/*
 * arch/arm/mach-imx/prs505.c
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/time.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/imx-uart.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <mach/i2c.h>

#include "devices-imx1.h"
#include "devices.h"

#define PRS505_FLASH_PHYS	MX1_CS0_PHYS
#define PRS505_FLASH_SIZE	MX1_CS0_SIZE

/*the bitwise definition of UCR1*/
#define SNDBRK		0x00000010
#define UART_CLK_EN	0x00000004
#define UARTEN		0x00000001
#define RRDYEN		0x00000200
#define IDEN		0x00001000

/*the bitwise definition of UCR2*/
#define IRTS	0x00004000
#define CTSC	0x00002000
#define CTS     0x00001000
#define PREN	0x00000100
#define PROE	0x00000080
#define STPB	0x00000040
#define WS      0x00000020
#define TXEN	0x00000004
#define RXEN	0x00000002
#define SRST	0x0000fffe

/*the interrupt enable bit*/
#define UCR1_TRDYEN		0x00002000
#define UCR1_TXMPTYEN	0x00000040

# define __REG(x)     (*((volatile u32 *)MX1_IO_ADDRESS(x)))

/*  08/5/27 */
#define _reg_PORTC_GIUS			GIUS(2)
#define _reg_PORTC_GPR			GPR(2)
#define _reg_UART1_URXD		__REG(MX1_UART1_BASE_ADDR + 0x0)
#define _reg_UART1_UTXD		__REG(MX1_UART1_BASE_ADDR + 0x40)
#define _reg_UART1_UCR1		__REG(MX1_UART1_BASE_ADDR + 0x80)
#define _reg_UART1_UCR2		__REG(MX1_UART1_BASE_ADDR + 0x84)
#define _reg_UART1_UCR3		__REG(MX1_UART1_BASE_ADDR + 0x88)
#define _reg_UART1_UCR4		__REG(MX1_UART1_BASE_ADDR + 0x8C)
#define _reg_UART1_UFCR		__REG(MX1_UART1_BASE_ADDR + 0x90)
#define _reg_UART1_USR1		__REG(MX1_UART1_BASE_ADDR + 0x94)
#define _reg_UART1_USR2		__REG(MX1_UART1_BASE_ADDR + 0x98)
#define _reg_UART1_UBIR			__REG(MX1_UART1_BASE_ADDR + 0xA4)
#define _reg_UART1_UBMR		__REG(MX1_UART1_BASE_ADDR + 0xA8)
#define _reg_AITC_INTSRCL		__REG(IMX_AITC_BASE + 0x4C)

/*dma transmitting*/
#define _reg_DMA_CCR1           CCR(1)
#define _reg_DMA_RSSR1         RSSR(1) 
#define _reg_DMA_BLR1           BLR(1)
#define _reg_DMA_RTOR1       	RTOR(1)
#define _reg_DMA_BUCR1        BUCR(1) 

#define _reg_DMA_DAR1          DAR(1) 
#define _reg_DMA_SAR1          SAR(1) 
#define _reg_DMA_CNTR1        CNTR(1)  

/*regsiters for dma*/
#define _reg_DMA_DIMR		DIMR

#define PRS505_NAND_CE0		(GPIO_PORTC | 14)
#define PRS505_NAND_BUSY	(GPIO_PORTC | 15)
#define PRS505_NAND_CLE		(GPIO_PORTC | 16)
#define PRS505_NAND_ALE		(GPIO_PORTC | 17)

#define PRS505_GPIO_USB_CHRG	(GPIO_PORTA | 6)
#define PRS505_GPIO_PCR5C	(GPIO_PORTA | 12)
#define PRS505_GPIO_SRR5C	(GPIO_PORTA | 10)
#define PRS505_GPIO_PCR5CCLK	(GPIO_PORTB | 9)
#define PRS505_GPIO_PCSDCARD	(GPIO_PORTB | 29)
#define PRS505_GPIO_PCMSCARD	(GPIO_PORTB | 28)

/*
 * UARTs platform data
 */

static struct imxuart_platform_data uart_pdata[] = {
	[0] = {
		.flags = IMXUART_HAVE_RTSCTS,
	},
	[1] = {
		.flags = 0,
	},
};


static const int nand_gpios[] = {PRS505_NAND_CE0, PRS505_NAND_BUSY,
	PRS505_NAND_CLE, PRS505_NAND_ALE};

static int prs505_nand_probe(struct platform_device *pdev)
{
	int i, err = 0;

	for (i = 0; i < ARRAY_SIZE(nand_gpios); i++)
		if (gpio_request(nand_gpios[i], "prs505-nand")) {
			err = -EBUSY;
			goto err_gpio_request;
		}

	gpio_direction_input(PRS505_NAND_BUSY);
	gpio_direction_output(PRS505_NAND_CE0, 0);
	gpio_direction_output(PRS505_NAND_CLE, 0);
	gpio_direction_output(PRS505_NAND_ALE, 0);

	return 0;

err_gpio_request:
	if (i)
		do {
			i--;
			gpio_free(nand_gpios[i]);
		} while (i);

	return err;
}

static void prs505_nand_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nand_gpios); i++)
		gpio_free(nand_gpios[i]);
}

static int prs505_nand_dev_ready(struct mtd_info *mtd)
{
	return gpio_get_value(PRS505_NAND_BUSY) ? 1 : 0;
}

static void prs505_nand_cmd_ctrl(struct mtd_info *mtd, int cmd,
		unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;

	if (ctrl & NAND_CTRL_CHANGE) {
		gpio_set_value(PRS505_NAND_CE0, (ctrl & NAND_NCE) ? 0 : 1);
		gpio_set_value(PRS505_NAND_CLE, (ctrl & NAND_CLE) ? 1 : 0);
		gpio_set_value(PRS505_NAND_ALE, (ctrl & NAND_ALE) ? 1 : 0);
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, chip->IO_ADDR_W);
}

static struct platform_nand_data prs505_nand_flash_data = {
	.chip	= {
		.nr_chips	= 1,
		.chip_delay	= 25,
		.part_probe_types = (const char *[]) {"SDM", NULL},
		.eccmode	= NAND_ECC_NONE,
	},
	.ctrl	= {
		.dev_ready	= prs505_nand_dev_ready,
		.cmd_ctrl	= prs505_nand_cmd_ctrl,
		.probe		= prs505_nand_probe,
		.remove		= prs505_nand_remove,
	},
};

static struct resource prs505_nand_resource[] = {
	[0] = {
		.start = MX1_CS1_PHYS,
		.end   = MX1_CS1_PHYS + MX1_CS1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	}
};


static struct platform_device prs505_device_nand = {
	.name		= "gen_nand",
	.id		= -1,
	.dev		= {
		.platform_data = &prs505_nand_flash_data,
	},
	.resource	= prs505_nand_resource,
	.num_resources	= ARRAY_SIZE(prs505_nand_resource),
};


static struct resource ebook_usb_s1r72v17_resources[] = {
	[0] = {
		.start	= 0x15E00000,
		.end	= 0x15E10000,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (IRQ_GPIOC(3)),
		.end	= (IRQ_GPIOC(3)),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource prs505_r5c807_sdhci_resources[] = {
	[0] = {
		.start	= 0x14e00100,
		.end	= 0x14e001ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (IRQ_GPIOA(11)),
		.end	= (IRQ_GPIOA(11)),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device prs505_sdhci = {
	.name		= "sdhci-r5c807-prs505",
	.resource	= prs505_r5c807_sdhci_resources,
	.num_resources	= ARRAY_SIZE(prs505_r5c807_sdhci_resources),
};

static struct resource prs505_nor_resource[] = {
	[0] = {
		.start = PRS505_FLASH_PHYS,
		.end   = PRS505_FLASH_PHYS + PRS505_FLASH_SIZE - 1,
		.flags = IORESOURCE_MEM,
	}
};

static struct mtd_partition prs505_nor_part[] = {
	[0] = {
		.name		= "Loader",
		.size		= 0x40000,
		.offset		= 0,
		.mask_flags	= MTD_WRITEABLE,
	},
	[1] = {
		.name		= "Wf",
		.size		= 0x40000,
		.offset		= 0x40000,
		.mask_flags	= MTD_WRITEABLE,
	},
	[2] = {
		.name		= "BootImg",
		.size		= 0x80000,
		.offset		= 0x80000,
		.mask_flags	= MTD_WRITEABLE,
	},
	[3] = {
		.name		= "Id",
		.size		= 0x10000,
		.offset		= 0x1c0000,
		.mask_flags	= MTD_WRITEABLE,
	},
	[4] = {
		.name		= "Info",
		.size		= 0x10000,
		.offset		= 0x1d0000,
		.mask_flags	= MTD_WRITEABLE,
	},
	[5] = {
		.name		= "FIS directory",
		.size		= 0x10000,
		.offset		= 0x1f0000,
		.mask_flags	= MTD_WRITEABLE,
	},
};


static struct physmap_flash_data prs505_nor_flash_data = {
	.width		= 2,
	.parts		= prs505_nor_part,
	.nr_parts	= ARRAY_SIZE(prs505_nor_part),
};

static struct platform_device prs505_device_nor = {
	.name		= "physmap-flash",
	.id		= -1,
	.dev		= {
		.platform_data = &prs505_nor_flash_data,
	},
	.num_resources	= ARRAY_SIZE(prs505_nor_resource),
	.resource	= prs505_nor_resource,
};

static struct platform_device ebook_usb_s1r72v17_device = {
	.name		= "s1r72v17",
	.num_resources	= ARRAY_SIZE(ebook_usb_s1r72v17_resources),
	.resource	= ebook_usb_s1r72v17_resources,
};

static struct platform_device *devices[] __initdata = {
	&ebook_usb_s1r72v17_device,
	&prs505_device_nor,
	&prs505_device_nand,
	&prs505_sdhci,
};

static void ebook_power_off(void)
{
    int i = 0;
    int trans_buf[5];
	unsigned long j;	/* 03/07/28 */

    printk(KERN_ERR "Shutdown ebook_power_off_new\n");  /* 03/07/28 */

    /* Clear the FIFO buffers and disable them  (they will be reenabled in change_speed())*/
    _reg_UART1_UCR2 &= SRST;
    /* delay for the reset*/
    for (i = 0; i < 1000; i++);

    _reg_UART1_UCR1 |= UARTEN;              /* Enable the UART */
    _reg_UART1_UCR2 |= (RXEN | TXEN);       /* 02/11/15 */
    _reg_UART1_UCR2 |= PREN;                /* 02/11/15 */
    _reg_UART1_UCR2 |= WS;                  /* 02/11/15 */

    /* not ignore RTS line */
    _reg_UART1_UCR2 &= 0x0000bfff;      /* IRTS=0 02/11/15 */
    /* CTS line control */
    _reg_UART1_UCR2 &= 0x0000dfff;      /* CTSC=0 02/11/15 */
    _reg_UART1_UCR2 |= CTS;             /* 02/11/15 */

    /*this is funny that UCR4 must be set as follows*/
    _reg_UART1_UCR4 = 1;

    /* and set the speed of the serial port */
    _reg_UART1_UBIR = 0xf;
    _reg_UART1_UBMR = 8000000 / 38400;  /* baudrate=38400 */
    _reg_UART1_UFCR = 0x00000A01;       /* 02/11/15 */
//    _reg_UART1_UFCR = 0x00000A81;

    /* delay for transmit data*/
    for (i = 0; i < 50000; i++);

    j = jiffies + 2;        /* 20ms wait */ /* 03/07/28 */
    while(jiffies < j){schedule();}         /* 03/07/28 */

    /* HOST Shutdown Complite COMMAND */
    trans_buf[0] = 0x000000D3;
    trans_buf[1] = 0x00000000;
    trans_buf[2] = 0x00000000;
    trans_buf[3] = 0x00000000;
    trans_buf[4] = 0x0000002C;

    for(i=0; i<5; i++){
        _reg_UART1_UTXD=trans_buf[i];
    };

    while(1);
}

static int prs505_pins[] = {
	PA22_PF_CS4,
	PA23_PF_CS5,
	PC9_PF_UART1_CTS,
	PC10_PF_UART1_RTS,
	PC11_PF_UART1_TXD,
	PC12_PF_UART1_RXD,
	PB30_PF_UART2_TXD,
	PB31_PF_UART2_RXD,
	GPIO_PORTA | GPIO_GPIO | GPIO_OUT | 18,
	GPIO_PORTA | GPIO_GPIO | GPIO_IN  | 19,
	GPIO_PORTA | GPIO_GPIO | GPIO_OUT | 20,
#ifdef CONFIG_LEDS
	GPIO_PORTA | GPIO_GPIO | GPIO_OUT | 2,
#endif
	PD30_PF_LD15,
	PD29_PF_LD14,
	PD28_PF_LD13,
	PD27_PF_LD12,
	PD26_PF_LD11,
	PD25_PF_LD10,
	PD24_PF_LD9,
	PD23_PF_LD8,
	PD22_PF_LD7,
	PD21_PF_LD6,
	PD20_PF_LD5,
	PD19_PF_LD4,
	PD18_PF_LD3,
	PD17_PF_LD2,
	PD16_PF_LD1,
	PD15_PF_LD0,
	PD14_PF_FLM_VSYNC,
	PD13_PF_LP_HSYNC,
	PD6_PF_LSCLK,
	PD12_PF_ACD_OE,
};

static void __init prs505_gpio_setup(void)
{
	int ret;

	mxc_gpio_setup_multiple_pins(prs505_pins,
		ARRAY_SIZE(prs505_pins), "prs505");

	ret = gpio_request(PRS505_GPIO_USB_CHRG, "USB_CHRG");
	if (ret)
		printk(KERN_INFO "Failed to request USB_CHRG GPIO\n");
	else
		gpio_direction_output(PRS505_GPIO_USB_CHRG, 1);
}

static void __init prs505_sdhci_init(void)
{
	void __iomem *base;
	struct clk *usb_clk;

	usb_clk = clk_get_sys("imx_udc.0", NULL);
	clk_enable(usb_clk);

	gpio_request(PRS505_GPIO_PCR5CCLK, "PCR5CCLK");
	gpio_direction_output(PRS505_GPIO_PCR5CCLK, 0);

	gpio_request(PRS505_GPIO_PCSDCARD, "PCSDCARD");
//	mxc_gpio_mode(PRS505_GPIO_PCSDCARD | GPIO_GPIO | GPIO_OUT);
	gpio_direction_output(PRS505_GPIO_PCSDCARD, 1);

	gpio_request(PRS505_GPIO_PCR5C, "PCR5C");
	gpio_direction_output(PRS505_GPIO_PCR5C, 1);

	gpio_request(PRS505_GPIO_SRR5C, "SRR5C");
	gpio_direction_output(PRS505_GPIO_SRR5C, 0);
	mdelay(10);
	gpio_set_value(PRS505_GPIO_SRR5C, 1);

	set_irq_type(IRQ_GPIOA(11), IRQF_TRIGGER_LOW);

	base = ioremap(MX1_CS3_PHYS, MX1_CS3_SIZE);
	if (!base) {
		printk(KERN_ERR "Failed to remap CS3\n");
		return;
	}

	writew(0x4444, base + 0x00e00000 + 0xF0);
	writew(0xc400, base + 0x00e00000 + 0xEE);
	writew(0x1008, base + 0x00e00000 + 0xF2);
	writew(0x0280, base + 0x00e00000 + 0xFA);

	iounmap(base);
}

static const struct imxi2c_platform_data prs505_i2c_data __initconst = {
	.bitrate = 100000,
};

static const struct i2c_board_info prs505_i2c_board_info[] = {
	{
		.type		= "lm75a",
		.addr		= 0x48,
	}
};

static void __init prs505_init(void)
{

	__REG(MX1_EIM_BASE_ADDR + 0x18) = 0x0000E000;
	__REG(MX1_EIM_BASE_ADDR + 0x1c) = 0x66660D01;

	prs505_gpio_setup();
	prs505_sdhci_init();

	/* UART */
	imx1_add_imx_uart0(&uart_pdata[0]);
	imx1_add_imx_uart1(&uart_pdata[1]);
	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* i2c */
	i2c_register_board_info(0, prs505_i2c_board_info, ARRAY_SIZE(prs505_i2c_board_info));
	imx1_add_imx_i2c(&prs505_i2c_data);

	pm_power_off = ebook_power_off;
}

static void __init prs505_timer_init(void)
{
	mx1_clocks_init(32768);
}

struct sys_timer prs505_timer = {
	.init	= prs505_timer_init,
};


MACHINE_START(SONY_PRS505, "Sony PRS-505 (Motorola DragonBall MX1)")
	/* Maintainer: Yauhen Kharuzhy */
	.boot_params	= 0x08000100,
	.map_io		= mx1_map_io,
	.init_irq	= mx1_init_irq,
	.timer		= &prs505_timer,
	.init_machine	= prs505_init,
MACHINE_END

