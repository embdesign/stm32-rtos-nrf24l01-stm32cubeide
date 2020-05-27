// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/compiler.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Compiler definitions
 */

#ifndef __MASU_COMPILER_H__
#define __MASU_COMPILER_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Assert */
#if !defined(PLATFORM_ASSERT)
#define PLATFORM_ASSERT(m_file, m_func, m_cond)
#endif

#define MASSERT_ARG(m_a)									\
	do {													\
		if (!(m_a)) { 										\
			PLATFORM_ASSERT(__FILE__, __func__, m_a);		\
		}										  			\
	} while (0)

#define MASSERT_RUN(m_a)									\
	do {													\
		if (!(m_a() { 										\
			PLATFORM_ASSERT(__FILE__, __func__, m_a);		\
		}										  			\
	} while (0)

/* IO access */
#define _IO 				volatile
#define _I  				volatile const
#define _O  				volatile

/* Types */

#ifdef __cplusplus
}
#endif

#endif /* __MASU_COMPILER_H__ */
