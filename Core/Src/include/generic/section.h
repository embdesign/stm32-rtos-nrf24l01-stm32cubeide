// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/section.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Linker sections
 */

#ifndef __MASU_SECTION_H__
#define __MASU_SECTION_H__

#ifdef __cplusplus
extern "C" {
#endif

/* References to section boundaries */
extern char __dm_driver_list_start[], __dm_driver_list_end[];
extern char __cmd_tbl_list_start[], __cmd_tbl_list_end[];

#ifdef __cplusplus
}
#endif

#endif /* __MASU_SECTION_H__ */
