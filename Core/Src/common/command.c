// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <common/command.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Command interpreter
 */

#include <common.h>
#include <string.h>
#include <command.h>

int cmd_is_argc_invalid(cmd_tbl_t *cmdtp, int argc, const char * const argv[])
{
	int help = 0;

	// Input argument check
	if ((argc < cmdtp->args_min) || (argc > cmdtp->args_max)) {
		help = 1;
	}

	if ((argc > 1) && (*argv[1] == '?')) {
		help = 1;
	}

	return help;
}

int cmd_exec(int argc, const char * const argv[])
{
	struct cmd_tbl *cmd;
	struct cmd_tbl *list_start = (struct cmd_tbl *)WARN_ASSUME_ALIGNED __cmd_tbl_list_start;
	struct cmd_tbl *list_end   = (struct cmd_tbl *)WARN_ASSUME_ALIGNED __cmd_tbl_list_end;
	int              ret         = -ENOENT;

	MASSERT_ARG(argv != 0);

	if (argc > 0) {
		while (list_start < list_end) {
			cmd = list_start;

			if (strncmp(cmd->name, argv[0], 20) == 0) {
				ret = cmd->cmd(cmd, 0, argc, argv);
				break;
			}

			list_start++;
		}
	}

	return ret;
}
