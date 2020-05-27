// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/driver/dm_device.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Device manager - device
 */

#ifndef __MASU_DM_DEVICE_H__
#define __MASU_DM_DEVICE_H__

#include <common.h>
#include <driver/dm_class-id.h>
#include <driver/dm_driver-id.h>

#ifdef __cplusplus
extern "C" {
#endif

struct dm_device;

/**
 * struct dm_driver - Generic driver
 */
struct dm_driver {
	char              *name;
	enum dm_driver_id driver_id; /* driver ID */
	enum dm_class_id  class_id;  /* class ID */
	int               (*bind)  (struct dm_device *dev);
	int               (*unbind)(struct dm_device *dev);
	int               (*probe) (struct dm_device *dev);
	int               (*remove)(struct dm_device *dev);
	const void        *ops;	     /* driver-specific operations */
};

#define DM_DRIVER_DEFINE(m_name) \
	const struct dm_driver _dm_driver_list_##m_name __attribute__((aligned(4))) __attribute__((unused, section(".dm_driver_list_"#m_name)))

/**
 * struct dm_device - An instance of a dm_driver
 */
struct dm_device {
	const struct dm_driver *driver;
	void                   *platdata;
};

int dm_device_is_valid(const struct dm_device *dev);
int dm_device_bind    (      struct dm_device *dev, enum dm_driver_id driver_id);
int dm_device_unbind  (      struct dm_device *dev);

#ifdef __cplusplus
}
#endif

#endif /* __MASU_DM_DEVICE_H__ */
