// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <driver/core/dm.c>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Device manager
 */

#include <common.h>
#include <driver/dm_device.h>

int dm_device_is_valid(const struct dm_device *dev)
{
	MASSERT_ARG(dev != NULL);

	return (dev->driver != NULL);
}

int dm_device_bind(struct dm_device *dev, enum dm_driver_id driver_id)
{
	struct dm_driver *driver;
	struct dm_driver *list_start = (struct dm_driver *)WARN_ASSUME_ALIGNED __dm_driver_list_start;
	struct dm_driver *list_end   = (struct dm_driver *)WARN_ASSUME_ALIGNED __dm_driver_list_end;
	int              status;

	MASSERT_ARG(dev != NULL);

	status = -ENODEV;

	while (list_start < list_end) {
		driver = list_start;

		if (driver->driver_id == driver_id) {
			status = 0;

			dev->driver = driver;

			if (driver->bind)
				status = driver->bind(dev);

			if (!status && driver->probe)
				status = driver->probe(dev);

			if (status) {
				if (driver->unbind)
					driver->unbind(dev);

				dev->driver = NULL;
			}

			goto probed;
		}

		list_start++;
	}

probed:
	return status;
}

int dm_device_unbind(struct dm_device *dev)
{
	const struct dm_driver *driver;
	int                    status;

	MASSERT_ARG(dev != NULL);

	status = 0;

	if (dm_device_is_valid(dev)) {
		driver = dev->driver;

		if (driver->remove)
			status = driver->remove(dev);

		if (driver->unbind)
			driver->unbind(dev);

		dev->driver = NULL;
	}

	return status;
}
