/*
 * drivers/mfd/sony-prs505-subcpu.c
 *
 * Driver for Sony PRS-505 bookreader subcpu (keyboard, LEDs, PM controller
 *
 * Copyright 2011 Yauhen Kharuzhy <jekhor@gmail.com>
 *	OpenInkpot Project: http://openinkpot.org/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>


#define DRIVER_DESC	"Driver for Sony PRS-505 subcpu"

struct prs505_subcpu_command {
	uint8_t opcode;
	uint8_t args[3];
	uint8_t cksum;
} __attribute__((packed));

const struct prs505_subcpu_command cmd_ack = {0xa2, {0, 0, 0}, 0x5d};
const struct prs505_subcpu_command cmd_nack = {0xa3, {0, 0, 0}, 0x5c};
const struct prs505_subcpu_command cmd_init = {0xd7, {0, 0, 0}, 0x28};
const struct prs505_subcpu_command cmd_req_bat_level = {0xb5, {0, 0, 0}, 0x4a};
const struct prs505_subcpu_command cmd_req_bat_status = {0xd1, {0, 0, 0}, 0x2e};

const unsigned int prs505_keymap[] = {
	[0x00] = KEY_UP,
	[0x01] = KEY_MENU,
	[0x02] = KEY_PAGEDOWN,
	[0x03] = KEY_SEARCH,
	[0x04] = KEY_PAGEUP,
	[0x05] = KEY_DOCUMENTS,
	[0x06] = KEY_KPMINUS,
	[0x07] = KEY_KPPLUS,
	[0x10] = KEY_1,
	[0x11] = KEY_2,
	[0x12] = KEY_3,
	[0x13] = KEY_4,
	[0x14] = KEY_5,
	[0x15] = KEY_6,
	[0x16] = KEY_7,
	[0x17] = KEY_8,
	[0x20] = KEY_LEFT,
	[0x21] = KEY_ENTER,
	[0x22] = KEY_DOWN,
	[0x23] = KEY_RIGHT,
	[0x24] = KEY_0,
	[0x25] = KEY_9,
	[0x27] = KEY_SCROLLUP,		/* side right */
	[0x26] = KEY_SCROLLDOWN,	/* side left */
};

struct prs505_subcpu_data {
	struct serio *serio;
	struct input_dev *input;
	unsigned char rx_buf[5];
	int rx_count;
	unsigned int battery_level;
};

static int prs505_subcpu_send_cmd(struct serio *serio, const struct prs505_subcpu_command *cmd)
{
	int i;
	unsigned char *p = (unsigned char *)cmd;

	for (i = 0; i < sizeof(*cmd); i++)
		serio_write(serio, p[i]);

	return 0;
}

static int prs505_subcpu_cmd_is_valid(unsigned char *cmd)
{
	return (cmd[0] + cmd[4] == 255);
}

static void prs505_subcpu_keypress(struct prs505_subcpu_data *prs505)
{
	int key = prs505->rx_buf[1];
	int pressed = (prs505->rx_buf[2] == 0x2b);

	if (key < ARRAY_SIZE(prs505_keymap)) {
		int keycode = prs505_keymap[key];

		if (keycode) {
			input_event(prs505->input, EV_KEY, keycode, pressed);
			input_sync(prs505->input);
		}
	}
}

static void prs505_subcpu_battery_level(struct prs505_subcpu_data *prs505)
{
	prs505->battery_level = prs505->rx_buf[2] & 0x1f;
}

static void prs505_subcpu_process_cmd(struct prs505_subcpu_data *prs505)
{
	int i;

	if (!prs505_subcpu_cmd_is_valid(prs505->rx_buf)) {
		prs505_subcpu_send_cmd(prs505->serio, &cmd_nack);
		return;
	}

	pr_debug("event: ");
	for (i = 0; i < 5; i++)
		pr_debug("%02x ", prs505->rx_buf[i]);

	pr_debug("\n");

	switch (prs505->rx_buf[0]) {
	case 0xB1:
		prs505_subcpu_keypress(prs505);
		break;
	case 0xd4:
		input_event(prs505->input, EV_KEY, KEY_POWER, 1);
		input_sync(prs505->input);
		input_event(prs505->input, EV_KEY, KEY_POWER, 0);
		input_sync(prs505->input);
		break;
	case 0xb4:
		prs505_subcpu_battery_level(prs505);
		break;
	default:
		break;
	}

	prs505_subcpu_send_cmd(prs505->serio, &cmd_ack);
}

static irqreturn_t prs505_subcpu_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
	struct prs505_subcpu_data *prs505 = serio_get_drvdata(serio);

	prs505->rx_buf[prs505->rx_count] = data;
	prs505->rx_count++;

	if (prs505->rx_count == 5) {
		prs505->rx_count = 0;
		prs505_subcpu_process_cmd(prs505);
	}

	return IRQ_HANDLED;
}

static int prs505_subcpu_connect(struct serio *serio, struct serio_driver *drv)
{
	struct prs505_subcpu_data *prs505;
	struct input_dev *input;
	int i, ret;

	prs505 = kzalloc(sizeof(*prs505), GFP_KERNEL);
	if (!prs505)
		return -ENOMEM;

	input = input_allocate_device();
	if (!input) {
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	input->evbit[0] = BIT(EV_KEY);

	input->name = "prs505-keys";
	input->phys = "prs505-keys/input0";

	input->id.bustype = BUS_RS232;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0001;

	for (i = 0; i < ARRAY_SIZE(prs505_keymap); i++) {
		if (prs505_keymap[i])
			input_set_capability(input, EV_KEY, prs505_keymap[i]);
	}

	input_set_capability(input, EV_KEY, KEY_POWER);

	ret = input_register_device(input);
	if (ret)
		goto err_input_register_device;

	prs505->serio = serio;
	prs505->input = input;

	serio_set_drvdata(serio, prs505);

	ret = serio_open(serio, drv);
	if (ret)
		goto err_serio_open;

	prs505_subcpu_send_cmd(serio, &cmd_init);
	prs505_subcpu_send_cmd(serio, &cmd_req_bat_status);
	prs505_subcpu_send_cmd(serio, &cmd_req_bat_level);

	return 0;

	serio_close(serio);
err_serio_open:
	input_unregister_device(input);
	input = NULL;
err_input_register_device:
	if (input)
		input_free_device(input);
err_input_alloc:
	serio_set_drvdata(serio, NULL);
	kfree(prs505);

	return ret;
}

static void prs505_subcpu_disconnect(struct serio *serio)
{
	struct prs505_subcpu_data *prs505 = serio_get_drvdata(serio);

	serio_close(serio);

	input_unregister_device(prs505->input);

	serio_set_drvdata(serio, NULL);
	kfree(prs505);
}

static struct serio_device_id prs505_subcpu_ids[] = {
	{
		.type   = SERIO_RS232,
		.proto  = SERIO_PRS505_SUBCPU,
		.id     = SERIO_ANY,
		.extra  = SERIO_ANY,
	},
	{ 0 }
};


static struct serio_driver prs505_subcpu_drv = {
	.driver = {
		.name	= "pr505-subcpu",
	},
	.description	= DRIVER_DESC,
	.id_table	= prs505_subcpu_ids,
	.interrupt	= prs505_subcpu_interrupt,
	.connect	= prs505_subcpu_connect,
	.disconnect	= prs505_subcpu_disconnect,
};

static int __init prs505_subcpu_init(void)
{
	return serio_register_driver(&prs505_subcpu_drv);
}

static void __exit prs505_subcpu_exit(void)
{
	serio_unregister_driver(&prs505_subcpu_drv);
}

module_init(prs505_subcpu_init);
module_exit(prs505_subcpu_exit);

MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
