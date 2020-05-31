// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <cmd/help.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Command interpreter
 */

#include <common.h>
#include <command.h>
#include <console.h>

int do_help(cmd_tbl_t *cmdtp, int flag, int argc, const char * const argv[])
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

	{
		const char   *name;
		const char   *help;
		unsigned int i = 0;

		while (cmd_get_info(i, &name, &help) == 0) {
			console_puts("\t");
			console_puts(name);
			console_puts(" - ");
			console_puts(help);
			console_puts(CONFIG_CONSOLE_LINE_ENDING_STRING);

			i++;
		}
	}

	return 0;
}

CMD_DEFINE(help) = {
	.name     = "help",
	.args_min = 1,
	.args_max = 1,
	.cmd      = do_help,
	.help     = "print build-in commands list",
};
