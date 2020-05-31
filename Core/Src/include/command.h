// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/command.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Command interpreter
 */

#ifndef __MASU_COMMAND_H__
#define __MASU_COMMAND_H__

#include <common.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cmd_tbl {
	char *name;    /* Command Name */
	int  args_min; /*
					* Minimum number of arguments
					* 0    invalid
					* 1    cmd name
					* 2..X cmd args
					*/
	int  args_max; /*
	                * Maximum number of arguments
	                */
	/* Implementation function */
	int  (*cmd)(struct cmd_tbl *cmd, int flags, int argc, const char * const argv[]);
	char *help;    /* Help message */
} cmd_tbl_t;

#define CMD_DEFINE(m_name)		\
	const struct cmd_tbl _cmd_tbl_list_##m_name __attribute__((aligned(4))) __attribute__((unused, section(".cmd_tbl_list_"#m_name)))

/* common/command.c */
int cmd_is_argc_invalid(cmd_tbl_t *cmdtp, int argc, const char * const argv[]);
int cmd_exec           (int argc, const char * const argv[]);
int cmd_get_info       (unsigned int cmd_id, const char **name, const char **help);

#ifdef __cplusplus
}
#endif

#endif /* __MASU_COMMAND_H__ */
