// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <common/shell.c>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Shell console
 */

#include <common.h>
#include <console.h>
#include <command.h>
#include <shell.h>

static int shell_filter_key_is_arrow(int c)
{
	static int is_block_character = 0;
	static int is_block_arrows    = 0;

	if (is_block_character) {
		is_block_character = 0;
		is_block_arrows    = 0;
		return 1;
	}

	if (c == 0x1B) {
		is_block_arrows = 1;
		return 1;
	}

	if ((is_block_arrows) && (c == 0x5B)) {
		is_block_character = 1;
		return 1;
	} else {
		is_block_arrows = 0;
	}

	return 0;
}

static inline int shell_filter_char_is_allowed(char input)
{
	int ret = 0;

	if ((input > 0x20) && (input < 0x7F)) {
		if ((input != CONFIG_SHELL_PROCESS_CMD_DELIMITER) &&
			(input != CONFIG_SHELL_PROCESS_ARG_DELIMITER)) {
			ret = 1;
		}
	}

	return ret;
}

static int shell_parse_cmd_from_line(char *line, int line_length, int *pargc, const char *argv[], int args_max)
{
	int parsed;

	*pargc = 0;
	parsed = 0;

	if (line_length == 0) goto empty_line;

	// Remove unsupported characters
	{
		char *src  = line;
		char *dest = line;
		int is_delimiter_allowed = 0;

		for (parsed = 0; parsed < line_length; parsed++) {
			if (*src == CONFIG_SHELL_PROCESS_CMD_DELIMITER) {
				parsed++;
				break;
			}

			if (shell_filter_char_is_allowed(*src)) {
				*dest++ = *src;

				is_delimiter_allowed = 1;
			} else {
				if (is_delimiter_allowed) {
					*dest++ = CONFIG_SHELL_PROCESS_ARG_DELIMITER;
				}

				is_delimiter_allowed = 0;
			}

			src++;
		}
		if ((line < dest) && (*(dest-1) == CONFIG_SHELL_PROCESS_ARG_DELIMITER)) {
			*(--dest) = '\0';
		} else {
			*(dest) = '\0';
		}
	}

	// Parse arguments
	{
		char *src = line;
		int is_argv = 1;

		while (*src != '\0') {
			if (*src == CONFIG_SHELL_PROCESS_ARG_DELIMITER) {
				*src = '\0';

				is_argv = 1;
			} else {
				if (is_argv) {
					argv[*pargc] = src;
					(*pargc)++;

					// No space left
					if (*pargc == args_max) {
						break;
					}

					is_argv = 0;
				}
			}

			src++;
		}
	}

empty_line:
	return parsed;
}

static int shell_process_line(char *line, int line_length)
{
	const char *argv[CONFIG_SHELL_PROCESS_ARG_CNT_MAX];
	int        argc;
	int        parsed;

	parsed = 0;

	while (line_length) {
		line = &line[parsed];
		parsed = shell_parse_cmd_from_line(line, line_length, &argc, argv, CONFIG_SHELL_PROCESS_ARG_CNT_MAX);
		line_length -= parsed;

		cmd_exec(argc, argv);
	}

	return 0;
}

void shell(void)
{
	int c;
	char line_buffer[CONFIG_SHELL_CMD_LINE_LENGTH_MAX];
	static int line_length = 0;

	console_puts(CONFIG_CONSOLE_LINE_ENDING_STRING "Command console (Type 'help'/'{command} ?' for help)" CONFIG_CONSOLE_LINE_ENDING_STRING);

	console_puts(CONFIG_SHELL_CMD_PROMPT_STRING);

	while (1) {
		c = console_getc();

		if (c > 0) {
			if (line_length < CONFIG_SHELL_CMD_LINE_LENGTH_MAX) {
				if (shell_filter_key_is_arrow(c)) {
					continue;
				}

				if (c == '\b') { // Backspace
					if (line_length) {
						line_length--;
						console_putc(c);
						console_putc(' ');
						console_putc(c);
					}
				} else {
					line_buffer[line_length++] = c;
					console_putc(c);
				}
			}

			if ((c == '\r') || (c == '\n')) {
				console_puts(CONFIG_CONSOLE_LINE_ENDING_STRING);

				shell_process_line(line_buffer, line_length);
				line_length = 0;

				console_puts(CONFIG_SHELL_CMD_PROMPT_STRING);
			}
		}
	}
}
