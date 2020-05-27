// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/config.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Configuration
 */

#ifndef __MASU_CONFIG_H__
#define __MASU_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

// Console
#define CONFIG_CONSOLE_LINE_ENDING_STRING		"\r\n"

// Data
#define CONFIG_DATA_VERSION_STRING				"1.0.0"

// Shell
#define CONFIG_SHELL_CMD_LINE_LENGTH_MAX		(64)
#define CONFIG_SHELL_CMD_PROMPT_STRING          "S > "
#define CONFIG_SHELL_PROCESS_ARG_CNT_MAX		(6)
#define CONFIG_SHELL_PROCESS_CMD_DELIMITER		';'
#define CONFIG_SHELL_PROCESS_ARG_DELIMITER		' '

#ifdef __cplusplus
}
#endif

#endif /* __MASU_CONFIG_H__ */
