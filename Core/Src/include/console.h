// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/console.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Console
 */

#ifndef __MASU_CONSOLE_H__
#define __MASU_CONSOLE_H__

#include <common.h>

#ifdef __cplusplus
extern "C" {
#endif

struct dm_device;

/**
 * enum console_flag - GD console flags
 */
enum console_flag {
	CONSOLE_FLAG__DEFAULT = 0ul,
	CONSOLE_FLAG__ENABLE  = BIT(1ul)
};

/**
 * struct console_ops - console ops
 */
struct console_ops {
	int (*getc)(const void *desc);
	int (*putc)(const void *desc, int c);
	int (*puts)(const void *desc, const char *s);
};

/**
 * struct console_desc - console instance
 */
struct console_desc {
	void               *desc;
	struct console_ops ops;
	enum console_flag  flags;
};

#define CONSOLE_ATTACH_DM_DEVICE(m_pdesc)	&((m_pdesc)->dev)

int console_init   (void);
int console_attach (const struct dm_device *dev);
int console_enable (void);
int console_disable(void);
int console_getc   (void);
int console_putc   (int c);
int console_puts   (const char *s);
int console_puti   (int num, int base);

#ifdef __cplusplus
}
#endif

#endif /* __MASU_CONSOLE_H__ */
