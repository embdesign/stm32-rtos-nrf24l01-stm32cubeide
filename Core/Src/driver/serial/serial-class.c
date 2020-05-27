// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <driver/serial/serial-class.c>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Serial class
 */

#include <common.h>
#include <driver/dm_device.h>
#include <driver/serial.h>

static inline int dm_serial_is_valid(const struct serial_desc *desc)
{
	return dm_device_is_valid(&desc->dev);
}

int dm_serial_init(struct serial_desc *desc, unsigned int port, enum dm_driver_id driver_id)
{
	int ret;

	ret = dm_device_bind(&desc->dev, driver_id);

	if (dm_serial_is_valid(desc)) {
		desc->port = port;
		ret        = 0;
	}

	return ret;
}

int dm_serial_deinit(struct serial_desc *desc)
{
	int ret;

	if (!dm_serial_is_valid(desc))
		return -ENOENT;

	ret = dm_device_unbind(&desc->dev);

	desc->port = 0u;

	return ret;
}

int dm_serial_getc(const struct serial_desc *desc)
{
	const struct dm_device     *dev;
	const struct dm_serial_ops *ops;
	int                    ret;

	if (!dm_serial_is_valid(desc))
		return -ENOENT;

	dev = &desc->dev;
	ops = serial_get_ops(dev);

	ret = ops->getc(dev, desc->port);

	return ret;
}

int dm_serial_getc_block(const struct serial_desc *desc)
{
	const struct dm_device     *dev;
	const struct dm_serial_ops *ops;
	int                    ret;

	if (!dm_serial_is_valid(desc))
		return -ENOENT;

	dev = &desc->dev;
	ops = serial_get_ops(dev);

	while ((ret = ops->getc(dev, desc->port)) == -EBUSY) {};

	return ret;
}

int dm_serial_putc(const struct serial_desc *desc, int c)
{
	const struct dm_device     *dev;
	const struct dm_serial_ops *ops;
	int                    ret;

	if (!dm_serial_is_valid(desc))
		return -ENOENT;

	dev = &desc->dev;
	ops = serial_get_ops(dev);

	ret = ops->putc(dev, desc->port, c);

	return ret;
}

int dm_serial_putc_block(const struct serial_desc *desc, int c)
{
	const struct dm_device     *dev;
	const struct dm_serial_ops *ops;
	int                    ret;

	if (!dm_serial_is_valid(desc))
		return -ENOENT;

	dev = &desc->dev;
	ops = serial_get_ops(dev);

	while ((ret = ops->putc(dev, desc->port, c)) == -EBUSY) {};

	return ret;
}

int dm_serial_puts(const struct serial_desc *desc, const char *s)
{
	const struct dm_device     *dev;
	const struct dm_serial_ops *ops;
	int                  ret;

	if (!dm_serial_is_valid(desc))
		return -ENOENT;

	dev = &desc->dev;
	ops = serial_get_ops(dev);
	ret = 0;

	{
		const char *c = s;

		while (*c) {
			while ((ret = ops->putc(dev, desc->port, *c)) == -EBUSY) {};
			c++;
		}
	}

	return ret;
}

int dm_serial_getconfig(const struct serial_desc *desc, uint32_t *config)
{
	const struct dm_device     *dev;
	const struct dm_serial_ops *ops;
	int                  ret;

	if (!dm_serial_is_valid(desc))
		return -ENOENT;

	dev = &desc->dev;
	ops = serial_get_ops(dev);

	ret = ops->getconfig(dev, desc->port, config);

	return ret;
}

int dm_serial_setconfig(const struct serial_desc *desc, uint32_t config)
{
	const struct dm_device     *dev;
	const struct dm_serial_ops *ops;
	int                  ret;

	if (!dm_serial_is_valid(desc))
		return -ENOENT;

	dev = &desc->dev;
	ops = serial_get_ops(dev);

	ret = ops->setconfig(dev, desc->port, config);

	return ret;
}
