// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <common/console.c>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Console
 */

#include <common.h>
#include <driver/dm_device.h>
#include <driver/serial.h>
#include <console.h>

static struct console_desc s_console = {0};

int console_init(void)
{
	struct console_desc *desc;
	int                 ret;

	desc = &s_console;
	ret  = 0;

	desc->desc     = NULL;
	desc->ops.getc = NULL;
	desc->ops.putc = NULL;
	desc->ops.puts = NULL;
	desc->flags    = CONSOLE_FLAG__DEFAULT;

	return ret;
}

int console_attach(const struct dm_device *dev)
{
	struct console_desc *cdesc;
	int                 ret;

	MASSERT_ARG(dev != NULL);

	cdesc = &s_console;
	ret   = -ENOENT;

	if (dm_device_is_valid(dev)) {
		switch (dev->driver->class_id) {
		case DM_CLASS_ID__SERIAL:
			{
				struct serial_desc *desc = CONTAINER_OF(dev, struct serial_desc, dev);

				cdesc->desc     = desc;
				cdesc->ops.getc = (int (*)(const void *desc)) dm_serial_getc_block;
				cdesc->ops.putc = (int (*)(const void *desc, int c)) dm_serial_putc_block;
				cdesc->ops.puts = (int (*)(const void *desc, const char *s)) dm_serial_puts;

				ret = 0;
			}
			break;

		default:
			break;
		}
	}

	return ret;
}

int console_enable(void)
{
	struct console_desc *desc;
	int                 ret;

	desc = &s_console;
	ret  = -ENOENT;

	if (desc->desc     &&
		desc->ops.getc && desc->ops.putc) {
		FLAG_ADD(desc->flags, CONSOLE_FLAG__ENABLE);
		ret = 0;
	}

	return ret;
}

int console_disable(void)
{
	struct console_desc *desc;
	int                 ret;

	desc = &s_console;
	ret  = 0;

	FLAG_DEL(desc->flags, CONSOLE_FLAG__ENABLE);

	return ret;
}

int console_getc(void)
{
	struct console_desc *desc;
	int                 ret;

	desc = &s_console;
	ret  = -EPERM;

	if (FLAG_IS_SET(desc->flags, CONSOLE_FLAG__ENABLE, CONSOLE_FLAG__ENABLE)) {
		ret = desc->ops.getc(desc->desc);
	}

	return ret;
}

int console_putc(int c)
{
	struct console_desc *desc;
	int                 ret;

	desc = &s_console;
	ret  = -EPERM;

	if (FLAG_IS_SET(desc->flags, CONSOLE_FLAG__ENABLE, CONSOLE_FLAG__ENABLE)) {
		ret = desc->ops.putc(desc->desc, c);
	}

	return ret;
}

int console_puts(const char *s)
{
	struct console_desc *desc;
	int                 ret;

	desc = &s_console;
	ret  = -EPERM;

	if (FLAG_IS_SET(desc->flags, CONSOLE_FLAG__ENABLE, CONSOLE_FLAG__ENABLE)) {
		ret = desc->ops.puts(desc->desc, s);
	}

	return ret;
}

#define CONSOLE_PUTI_STRING_LENGTH_MAX	(11)
int console_puti(int num, int base)
{
	unsigned int uvalue;
	unsigned int fnegative;
	char         str[CONSOLE_PUTI_STRING_LENGTH_MAX+1];
	char         *p_str = str;
	char         *p_str_end = &str[CONSOLE_PUTI_STRING_LENGTH_MAX];
	int          ret;

	if ((num < 0) && (base == 10)) {
		fnegative = 1u;
		uvalue = -num;
	} else {
		fnegative = 0u;
		uvalue = num;
	}

	if (uvalue == 0u) {
		*p_str++ = '0';
	}

	while ((p_str != p_str_end) && (uvalue != 0u)) {
		int rem = uvalue % base;
		*p_str++ = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
		uvalue = uvalue / base;
	}

	if (fnegative) {
		*p_str++ = '-';
	}

	*p_str = '\0';

	// Reverse string
	{
		char *p_start = str;
		char *p_end   = --p_str;

		while (p_start < p_end) {
			  *p_start ^= *p_end;
			  *p_end   ^= *p_start;
			  *p_start ^= *p_end;

			  p_start++;
			  p_end--;
		}
	}

	ret = console_puts(str);

	return ret;
}
