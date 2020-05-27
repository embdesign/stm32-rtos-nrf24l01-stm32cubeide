// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/driver/dm_driver-id.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Device manager - driver id
 */

#ifndef __MASU_DM_DRIVER_ID_H__
#define __MASU_DM_DRIVER_ID_H__

#ifdef __cplusplus
extern "C" {
#endif

enum dm_driver_id {
	DM_DRIVER_ID__EMPTY          = 0x00000000l,
	DM_DRIVER_ID__STM32F10X_UART = 0x00010010l,
};

#ifdef __cplusplus
}
#endif

#endif /* __MASU_DM_DRIVER_ID_H__ */
