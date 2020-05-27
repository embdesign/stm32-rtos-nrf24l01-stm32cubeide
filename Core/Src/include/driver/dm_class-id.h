// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/driver/dm_class-id.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Device manager - class id
 */

#ifndef __MASU_DM_CLASS_ID_H__
#define __MASU_DM_CLASS_ID_H__

#ifdef __cplusplus
extern "C" {
#endif

enum dm_class_id {
	DM_CLASS_ID__EMPTY  = 0x00000000l,
	DM_CLASS_ID__SERIAL = 0x00000010l
};

#ifdef __cplusplus
}
#endif

#endif /* __MASU_DM_CLASS_ID_H__ */
