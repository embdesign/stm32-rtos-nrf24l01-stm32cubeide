// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <cmd/version.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Command interpreter
 */

#include <common.h>
#include <command.h>
#include <console.h>

int do_version(cmd_tbl_t *cmdtp, int flag, int argc, const char * const argv[])
{
	//WARN_UNUSED(cmdtp);
	WARN_UNUSED(flag);
	//WARN_UNUSED(argc);
	//WARN_UNUSED(argv);

	// Help
	if (cmd_is_argc_invalid(cmdtp, argc, argv)) {
		console_puts(cmdtp->name);
		console_puts(" - ");
		console_puts(cmdtp->help);
		console_puts(CONFIG_CONSOLE_LINE_ENDING_STRING CONFIG_CONSOLE_LINE_ENDING_STRING);
		console_puts("Prototype:" CONFIG_CONSOLE_LINE_ENDING_STRING "\t");
		console_puts(cmdtp->name);
		console_puts(CONFIG_CONSOLE_LINE_ENDING_STRING);
		console_puts("Example:" CONFIG_CONSOLE_LINE_ENDING_STRING "\t");
		console_puts(cmdtp->name);
		console_puts(CONFIG_CONSOLE_LINE_ENDING_STRING);

		return 0;
	}


	console_puts(CONFIG_DATA_VERSION_STRING CONFIG_CONSOLE_LINE_ENDING_STRING);

	return 0;
}

CMD_DEFINE(version) = {
	.name     = "version",
	.args_min = 1,
	.args_max = 1,
	.cmd      = do_version,
	.help     = "version info",
};
